### execution result
* 1 no noise (noise_w == 0.0)
```
(base) brucesz@sbrucezhang-LB1:~/git/localization_slam/vio_assign/course6_hw/build$ ./estimate_depth 
Singular values: 
     1.56875
    1.48736
   0.084117
3.38629e-17
ratio of smallest singular value and second smallest singular value : 4.02569e-16
ground truth: 
  -2.9477 -0.330799   8.43792
your result: 
  -2.9477 -0.330799   8.43792


```

* 2 plot with different noise_w.
```
 for i in $(seq 0.1 0.1 5);do echo -n $i, ":"  ;./estimate_depth  -noise_var=$i -noise_w=1  -n_frame=6|grep ratio ;done  > noise_var_ratio

```

* 3 plot with different frame with fixed noise_w==2.0
```
for i in $(seq 3 96);do echo -n $i, ":"  ;./estimate_depth  -noise_var=2.0 -noise_w=1  -n_frame=$i|grep ratio ;done  > frame_ratio
```


