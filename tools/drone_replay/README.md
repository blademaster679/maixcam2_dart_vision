# 無人機綠燈離線回放

本工具在 `slam_drone` 整合上游 `fd7ff396994f0da69c67573ef4b52c531abc1e6b`。
輸入是帶拍攝時間的 RGB 影像序列，使用原 C++ NV21 候選搜尋、
`measure_fast_green`、`ModelConfirmationGate` 與 `CenterKalman`。
RGB 經 OpenCV I420→NV21 適配供候選搜尋；局部質心直接從原 RGB 量測。
板端 NV21 轉 RGB 與此主機適配可能有像素差異，尚無板端逐像素等價證據。

模型用來確認候選是否為綠燈，當幀中心仍由像素量測取得。模型中心必須與
候選相距不超過 `max(12 px, diameter)`，才計為肯定；兩次同一軌跡肯定後
開始输出。既有穩定軌跡允許一次模型否定，但肯定不會因此刷新，兩次否定
即失鎖；沒有當幀像素就不輸出量測。沒有裝甲辨識、位姿、純預測中心或控制輸出。

無人機配置將允許的組件長寬比設為 8：現有斜視仿真標記首幀的亮核心為
9×2 像素，原預設 4 會拒絕它。共用函數原預設仍為 4。
中心濾波最大間隔和肯定有效期為兩個來源幀週期（20 Hz 時 100 ms）；
原 CenterKalman 預設仍為 50 ms。搜尋使用實際影像邊界，沒有 ±6.5° 捕獲錐。
這些是回放研發配置，未作實機資格判定。

## 模型與 Pulsar2

目前使用 `/home/shiuhou/Projects/dart_target_pose.mud` 及其引用的
`dart_target_pose_npu.axmodel`、`dart_target_pose_vnpu.axmodel`。三個檔案留在本地。
NPU 版本輸入：`uint8 [1,320,320,3]` RGB。將縮放／填充後的 RGB bytes
直接送入模型，不能另外除以 255。輸出為三個 NHWC 65 通道 DFL 檢測頭，
及 `[1,2100,15]` Pose 張量。只讀关键點 0，五個關鍵點輸出並不能證明
五個點都經過有效監督。

YOLO11 後處理對照 MaixCDK `maix_nn_yolo11.hpp` 的 DFL、sigmoid、NMS
及關鍵點解碼公式。主機實作不是板端 NPU API；量化模型由 Pulsar2 模擬器
實際執行。綠燈中心回放的信心門檻採上游 `board_center_replay.py` 和
`board_center_validator.cpp` 的 0.3，關鍵點門檻 0.35、NMS 0.45 保持一致。
這是上游離線回放配置；作者尚未提供此仿真相機的實機部署門檻證據。

官方工具下載：[Pulsar2 6.0](https://huggingface.co/AXERA-TECH/Pulsar2/tree/main/6.0)。
本機使用 `ax_pulsar2_6.0_lite_package.tar.gz`，SHA256
`83e38b39668729b5a144531dc733a0d3a0a336734e18d8a0fcb4ae6a65e06dec`，
工具報告 `version: 6.0 / commit: 48520c11`，匹配模型內編譯資訊。
解壓位置在 PX4 工作區的 `artifacts/deps/pulsar2-6.0/`。
官方套件 `bin/python3` 引用缺失的 `ld-2.31.so`；模型檢查直接使用套件內
`ld-linux-x86-64.so.2` 和 Python，不修改系統或供應商套件。

## 建置與回放

在此視覺倉庫根目錄執行：

```bash
cmake -S tools/drone_replay -B artifacts/drone-core-build-new -DCMAKE_BUILD_TYPE=Release
cmake --build artifacts/drone-core-build-new -j 4

/home/shiuhou/venvs/mujoco/bin/python tools/drone_replay/replay.py \
  --manifest /absolute/path/to/rgb-input/manifest.json \
  --mud /home/shiuhou/Projects/dart_target_pose.mud \
  --toolchain /home/shiuhou/Projects/rm27_drones_px4/artifacts/deps/pulsar2-6.0/ax_pulsar2_6.0_lite_package \
  --library artifacts/drone-core-build-new/libdrone_green_core.so \
  --output artifacts/drone-replay-new --expect-target present
```

`--limit N` 只處理前 N 幀；`--expect-target present` 要求至少兩幀有效輸出，
`absent` 要求零有效輸出。這是有限功能檢查，未評估整個輸入的精度或可靠性。
`unspecified` 僅檢查流程，不能代表追蹤通过。所有輸出目錄必須不存在。

Pulsar2 將每個候選裁切作為獨立推理輸入；40 幀回放可能需要數分鐘。
`model_timeout_s` 可於獨立配置調整，預設 600 秒。來源幀率只用於時間和影片，
不能解讀成主機、板端或實機即時幀率。

## RGB manifest 和輸出

`manifest.json` 欄位：`schema_version: 1`、`pixel_format: RGB888`、
`width`、`height`、`nominal_fps`、`clock_domain`、`frames`。
每一幀有 `sequence`、整數微秒 `timestamp_us`、相對圖片路徑 `image` 和 `sha256`。
圖片以 PNG 保存，OpenCV 載入後轉為 RGB；灰階圖不能代替此輸入。
反向／重複來源時間使當幀無效並清除追蹤，不以影像處理速度重建拍攝時間。

產物：`annotated.mp4`、`results.jsonl`、`summary.json`、`model_io.json`、
來源快照、模型／設定／函式庫哈希、模型輸入／原始輸出及 `pulsar2.log`。
橙色圈為候選，綠色十字為已確認的平滑量測。`safe_for_control` 始終為 false，
避免離線結果被當作已合格飛行輸入。

`inference_manifest.json` 保存每份模型輸入與輸出 SHA256。
`--reuse-inference PREVIOUS_OUTPUT` 僅允許相同模型、工具版本、I/O 定義和
完全相同的準備後 RGB bytes，並核對輸出張量哈希。此模式在摘要中標為
`recorded AXMODEL tensor replay`，另報新推理數與重用數；不宣稱再次執行了模型。

## 遮擋與測試

```bash
/home/shiuhou/venvs/mujoco/bin/python tools/drone_replay/fault_manifest.py \
  --manifest /absolute/path/to/rgb-input/manifest.json \
  --output artifacts/blackout-input-new --mode blackout --limit 40 --first 10 --count 6

DRONE_CORE_LIBRARY="$PWD/artifacts/drone-core-build-new/libdrone_green_core.so" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python \
  -m pytest tools/drone_replay/test_replay.py -q
```

遮擋 fixture 只替換指定 RGB 為黑圖，保留來源時間和原始輸入哈希；原圖與場景不變。
`--mode no_target` 產生全黑輸入，驗證沒有像素候選時保持無效，不能證明模型能
拒絕所有真实干擾物。合成模型否定與座標變化另由接口測試驗證。

`--mode blackout_reacquire` 在黑圖後重複前面的錄製視角，並保留 fixture 的
單調來源時間。這是明確的合成重現測試，不是第二次飛行或自然遮擋恢復證據。
`audit.py` 獨立核對預設第 10–15 幀黑圖區間與重獲；只適用預設 fixture 邊界。

2026-10-14 核對發現先前 0.5 取自通用 `config/green_detector.conf`，
作者的中心離線回放工具則使用 0.3。本工具現對齊後者；實際板端部署配置
尚未取得，不能把這個修正當作實機配置已確認。裁切規則和模型保持原樣。

相同 40 幀／模型張量的結果如下；這些計數沒有人工標註，不能當作準確率。

| 配置 | 模型肯定 | 有效當幀追蹤輸出 | 黑圖後恢復原視角序列 |
| --- | --- | --- | --- |
| 舊通用門檻 0.5 | 14/40 | 17/40 | 未重獲 |
| 上游中心回放門檻 0.3 | 27/40 | 28/40 | 第二幀重新確認 |

有效輸出可在一次模型否定後短暫保留，所以兩種計數不同。新版在第 29–39 幀
持續失鎖；全部 40 幀都有像素候選，13 次模型拒絕均由分類信心不足引起，
並非這批输入的關鍵點座標還原或關聯失敗。連續追蹤仍未通過。
畫面與裁切大小會一起變動，目前不能據此判定是裁切問題或訓練資料差異。

新結果為精確輸入的錄製 AXMODEL 張量回放，重用先前真正執行的 40 次推理；
本次新推理數為零。`artifacts/drone-replay-center-audit-01/summary.json`
確認黑圖期間無輸出、原視角恢復和明確合成重現均能重新確認、無目標片段
保持無效。9 項接口／解碼測試通過。黑圖結果不代表真實干擾物的誤檢率。

診斷入口可輸出實際 320×320 模型輸入、逐幀分數、肯定／拒絕對照圖與
固定上游來源快照／哈希：

```bash
/home/shiuhou/venvs/mujoco/bin/python tools/drone_replay/diagnose.py \
  --run artifacts/drone-green-nominal-01 \
  --output artifacts/drone-green-diagnose-new
```

`--run` 必須含完整模型 I/O、jobs、輸入 bytes、輸出張量及
`inference_manifest.json`。此診斷對照兩個既有來源門檻，不搜尋最佳門檻。
新證據索引位於 PX4 工作區 `artifacts/green-replay-diagnosis-closeout-01/summary.json`；
舊輪次與失敗嘗試另行保留。後續需要實機部署配置及帶標註的真實影像／
訓練資料，才能檢查模型為何拒絕末段仿真視角。

距離、裝甲識別、動態飛行控制、板端即時調度與 OS04A10 成像驗證均待後續研究。

## 固定影像的 NV21 輸入診斷（2026-10-15）

新增 `input_probe.py`：從錄製推理固定候選及 ROI，將原 RGB 與「主機產生
NV21、再經原 C++ `nv21_rgb_region` 轉回 RGB」作配對推理。此轉換只用於
診斷，不改預設回放、場景或控制。它不是實機感測器的原始 NV21，不能代表
板端成像表現；局部質心和 ROI 故意固定，以單獨檢查模型輸入的影響。

```bash
cmake -S tools/drone_replay -B artifacts/drone-probe-build-new -DCMAKE_BUILD_TYPE=Release
cmake --build artifacts/drone-probe-build-new -j 4
/home/shiuhou/venvs/mujoco/bin/python tools/drone_replay/input_probe.py \
  --baseline artifacts/drone-green-nominal-01 \
  --manifest /home/shiuhou/Projects/rm27_drones_px4/artifacts/green-rgb-replay-01/manifest.json \
  --toolchain /home/shiuhou/Projects/rm27_drones_px4/artifacts/deps/pulsar2-6.0/ax_pulsar2_6.0_lite_package \
  --library artifacts/drone-probe-build-new/libdrone_green_core.so \
  --frames 8 26 30 39 --output artifacts/drone-input-probe-new
```

推理前保存 `registration.json`：固定幀、候選、門檻、素材與程式哈希。
基線張量須通過模型／工具版本、I/O、原影像、準備後 RGB bytes 及輸出哈希
檢查；變體實際執行 AXMODEL。輸出 `comparison.png`、`per_frame.json`、
原始張量和摘要。`passed` 僅表示診斷完成，沒有連續追蹤或飛行資格含義。

四張預先固定影像的最大分類信心如下，門檻維持 0.3：

| 幀 | 原 RGB（重用推理） | NV21 轉回 RGB（新推理） |
| --- | --- | --- |
| 8 | 0.74155，通過中心關聯 | 0.25845，拒絕 |
| 26 | 0.04062，拒絕 | 0.01454，拒絕 |
| 30 | 0.10832，拒絕 | 0.00864，拒絕 |
| 39 | 0.00864，拒絕 | 0.00179，拒絕 |

沒有恢復三張低信心影像，且原本通過的一張也被拒絕，因此沒有將 NV21
轉換設為預設。有限對照顯示輸入像素影響模型信心，尚不能區分場景背景、
渲染細節、裁切尺度或訓練資料等原因。模型是否依賴裝甲背景仍未知。

另固定官方 MaixCDK `30f4b8b7e3f66ded9cfa64fb081b46e30bc27248`，核對
NHWC 分類 sigmoid、DFL、關鍵點 anchor／stride 和整數截斷公式一致。
部署端 SDK 版本、NMS 和浮點逐像素等價仍未認證。
原碼與檢查保存於 `artifacts/drone-input-audit-01/`。

新推理 4 次、重用基線 4 次，10 項相關测试通過。
`artifacts/drone-nv21-paired-probe-02/summary.json` 記錄完成結果；首輪因
舊資料沒有獨立 `model_bundle.json` 而中止，保留於 `-01`。入口改讀已有的
`inference_manifest.json` 模型哈希後完成，沒有略過模型核對。
本輪沒有改門檻、重訓、增加飛行批次或宣稱末段失鎖已解決。
