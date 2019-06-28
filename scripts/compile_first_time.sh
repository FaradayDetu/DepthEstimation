#!/bin/bash
cd ..
source ~/tensorflow/bin/activate
cd ~/redtail/stereoDNN/scripts
python ./model_builder.py --model_type resnet18_2D --net_name ResNet18_2D_513x257 --checkpoint_path=../models/ResNet-18_2D/TensorFlow/model-inference-513x257-0 --weights_file=../models/ResNet-18_2D/TensorRT/trt_weights.bin --cpp_file=../sample_app/resnet18_2D_513x257_net.cpp --data_type fp32
cd ~/redtail/stereoDNN/build
rm -r * 
cmake -DCMAKE_BUILD_TYPE=Debug ..
make 
cd ~/catkin_ws

