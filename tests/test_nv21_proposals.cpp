#include "dart/nv21_pipeline.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

namespace {
using Color=std::array<int,3>;
int failures=0;
void check(bool value,const char *message) {
    if(!value){++failures;std::cerr<<"FAIL: "<<message<<'\n';}
}
struct Scene {
    int w,h,stride;
    std::vector<Color> rgb;
    std::vector<uint8_t> y,vu;
    Scene(int width=320,int height=240,Color fill={20,20,20})
        :w(width),h(height),stride(width+16),rgb(width*height,fill),y(stride*height,0),vu(stride*height/2,0){}
    static uint8_t clamp(int value){return static_cast<uint8_t>(std::clamp(value,0,255));}
    dart::Nv21View view() {
        for(int yy=0;yy<h;++yy) for(int xx=0;xx<w;++xx) {
            const auto &c=rgb[yy*w+xx];
            y[yy*stride+xx]=clamp(((66*c[0]+129*c[1]+25*c[2]+128)>>8)+16);
        }
        for(int yy=0;yy<h;yy+=2) for(int xx=0;xx<w;xx+=2) {
            Color c={0,0,0};
            for(int dy=0;dy<2;++dy) for(int dx=0;dx<2;++dx)
                for(int channel=0;channel<3;++channel)c[channel]+=rgb[(yy+dy)*w+xx+dx][channel];
            for(auto &channel:c)channel=(channel+2)/4;
            vu[(yy/2)*stride+xx]=clamp(((112*c[0]-94*c[1]-18*c[2]+128)>>8)+128);
            vu[(yy/2)*stride+xx+1]=clamp(((-38*c[0]-74*c[1]+112*c[2]+128)>>8)+128);
        }
        return {y.data(),vu.data(),w,h,stride,stride};
    }
    void spot(float cx,float cy,float radius,Color color) {
        for(int yy=0;yy<h;++yy) for(int xx=0;xx<w;++xx)
            if(std::hypot(xx-cx,yy-cy)<=radius)rgb[yy*w+xx]=color;
    }
};
bool near(const std::vector<dart::Point2f>&points,float x,float y,float radius) {
    return std::any_of(points.begin(),points.end(),[&](const auto&p){return std::hypot(p.x-x,p.y-y)<=radius;});
}
}
int main(){
    // Sweep scale, brightness and all phases of chroma downsampling. These are
    // geometric/color fixtures, independent of recording coordinates or frames.
    for(int green:{60,120,240}) for(int radius:{1,2,4,8,14})
        for(int px:{0,1}) for(int py:{0,1}) {
            Scene scene;const int x=121+px,y=91+py;
            scene.spot(x,y,radius,{10,green,10});
            const auto proposals=dart::nv21_green_proposals(scene.view());
            check(near(proposals,x,y,radius+2),"compact dim/bright green lamp survives scale/chroma phase");
        }
    for(int green:{180,240}) for(int px:{0,1}) for(int py:{0,1}) {
        Scene scene;const int x=111+px,y=81+py;
        scene.rgb[y*scene.w+x]={0,green,0};
        check(near(dart::nv21_green_proposals(scene.view()),x,y,2),
              "single bright source pixel survives shared 2x2 chroma");
    }
    for(int green:{80,160,240}) {
        Scene scene;
        for(int y=0;y<scene.h;++y) for(int x=0;x<scene.w;++x) {
            const float distance2=(x-153)*(x-153)+(y-115)*(y-115);
            scene.rgb[y*scene.w+x][1]+=static_cast<int>(green*std::exp(-distance2/24));
        }
        check(near(dart::nv21_green_proposals(scene.view()),153,115,5),
              "soft optical falloff retains a green peak without a hard component edge");
    }
    for(int radius:{3,6,12}) {
        Scene scene;scene.spot(157,117,radius,{20,240,30});
        scene.spot(157,117,radius/2,{255,255,255});
        const auto proposals=dart::nv21_green_proposals(scene.view());
        check(near(proposals,157,117,radius+2),"saturated white core retains its green fringe proposal");
    }
    for(const auto color:{Color{20,180,20},Color{40,180,170},Color{180,190,20}}) {
        Scene uniform(320,240,color);
        check(dart::nv21_green_proposals(uniform.view()).empty(),"uniform green/cyan/yellow surfaces have no local proposal");
        for(int angle=0;angle<180;angle+=15) {
            Scene edge;const float a=angle*3.14159265358979323846F/180;
            for(int y=0;y<edge.h;++y)for(int x=0;x<edge.w;++x)
                if((x-160)*std::cos(a)+(y-120)*std::sin(a)>=0)edge.rgb[y*edge.w+x]=color;
            check(dart::nv21_green_proposals(edge.view()).empty(),"rotated flat color edge does not consume a proposal");
        }
    }
    {
        // Distractor surfaces occupy many old 64px tile maxima; the small lamp
        // should be available regardless of their total image area.
        Scene scene(640,480);
        for(int y=220;y<scene.h;++y)for(int x=0;x<scene.w;++x)scene.rgb[y*scene.w+x]={50,170,150};
        scene.spot(105,99,2,{15,160,25});
        const auto proposals=dart::nv21_green_proposals(scene.view());
        check(!proposals.empty() && std::hypot(proposals[0].x-105,proposals[0].y-99)<6,
              "tiny lamp ranks ahead of broad bright chromatic background");
    }
    {
        Scene scene(640,480);
        for(int row=0;row<3;++row)for(int col=0;col<6;++col)scene.spot(40+100*col,40+150*row,3,{10,200,20});
        const auto proposals=dart::nv21_green_proposals(scene.view());
        check(proposals.size()==5,"global search keeps bounded five-proposal budget");
        for(std::size_t i=0;i<proposals.size();++i)for(std::size_t j=0;j<i;++j)
            check(std::hypot(proposals[i].x-proposals[j].x,proposals[i].y-proposals[j].y)>=48,
                  "proposal suppression remains in source pixels");
        const auto repeat=dart::nv21_green_proposals(scene.view());
        check(repeat.size()==proposals.size(),"proposal ordering repeat count is stable");
        for(std::size_t i=0;i<proposals.size();++i)
            check(proposals[i].x==repeat[i].x && proposals[i].y==repeat[i].y,"equal-score ordering is deterministic");
    }
    {
        Scene scene(1344,760,{30,110,100});scene.spot(891,101,4,{20,230,30});
        for(int scenario=0;scenario<2;++scenario) {
            if(scenario==1) {
                // Dense green texture exercises the upper search-work case;
                // report cost without claiming a hardware-independent limit.
                for(int y=0;y<scene.h;++y) for(int x=0;x<scene.w;++x)
                    scene.rgb[y*scene.w+x]={10,((x/2+y/2)%2)?220:240,10};
            }
            const auto view=scene.view();
            const auto started=std::chrono::steady_clock::now();
            for(int i=0;i<30;++i)(void)dart::nv21_green_proposals(view);
            const auto us=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-started).count();
            std::cout<<"proposal microbenchmark (host/board depends on executable; not detector FPS), "
                     <<(scenario?"dense texture":"surface and lamp")<<": "<<us/30000.0<<" ms/call\n";
        }
    }
    if(failures){std::cerr<<failures<<" failures\n";return 1;}
    std::cout<<"NV21 proposal tests passed\n";
}
