# !ATTENTION! Code écrit en chinois en partie. Impact sur la programmation ? 


# Header (std_msgs/Header): Timestamp, frame_id is not currently used.

# Left-hand tactile data. Total length is 1150 values, with 1086 valid values.
float32[] left

# Right-hand tactile data. Total length is 1150 values, with 1086 valid values.
float32[] right

# Each tactile point occupies 3 consecutive elements.


# La suite décrit la position des valeurs de force X, Y, Z dans un tableau
# Par ex si j=0 on a X0, Y0, Z0
# De même si j=3 on a X3, Y3, Z3
# Pas hyper clair -> à voir en utilisation 
# [3j + 0]: tangential force along the X direction (original type: int8, cast to float32, range: -12.8 to 12.7 N)
[3j+0]：X ⽅向切向⼒（原始类型 int8, cast 为 float32，范围：-12.8~12.7N）

# [3j + 1]: tangential force along the Y direction (original type: int8, cast to float32, range: -12.8 to 12.7 N)
[3j+1]：Y ⽅向切向⼒（原始类型 int8，cast 为 float32，范围：-12.8~12.7N）

# [3j + 2]: normal contact force (original type: uint8, cast to float32, range: 0 to 25.5 N)
[3j+2]：法向压⼒（原始类型 uint8，cast 为 float32，范围：0~25.5N）