 ## gen noise_var_ratio data

 * 1 for i in $(seq 3 96);do echo -n $i, ":"  ;./estimate_depth  -noise_var=$i -noise_w=1  -n_frame=10|grep ratio ;done  > noise_var_ratio

 ## gen frame_ratio data
 * 2 for i in $(seq 3 96);do echo -n $i, ":"  ;./estimate_depth  -noise_var=100 -noise_w=1  -n_frame=$i|grep ratio ;done  > frame_ratio