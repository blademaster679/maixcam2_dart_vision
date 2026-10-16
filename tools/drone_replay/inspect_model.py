"""Run with the Pulsar2 distribution's Python; read compiled graph I/O only."""
import json
import sys
import onnx

model = onnx.load_model(sys.argv[1], load_external_data=False)
def describe(values):
    return [dict(name=v.name, dtype=v.type.tensor_type.elem_type,
                 shape=[d.dim_value for d in v.type.tensor_type.shape.dim]) for v in values]
print(json.dumps(dict(inputs=describe(model.graph.input), outputs=describe(model.graph.output))))
