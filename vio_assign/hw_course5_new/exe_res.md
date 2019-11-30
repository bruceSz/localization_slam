## some execution result
Note: we also compute abs error by subtracting opt  with gt .
## execution result with no prior(without adding prior edge into the problem).
1
```
(base) brucesz@sbrucezhang-LB1:~/git/localization_slam/vio_assign/hw_course5_new/build$  ./app/testMonoBA  
0 order: 0
1 order: 6
2 order: 12

 ordered_landmark_vertices_ size : 20
iter: 0 , chi= 5.35099 , Lambda= 0.00597396
iter: 1 , chi= 22.6734 , Lambda= 0.00199132
iter: 2 , chi= 103.343 , Lambda= 0.000663774
iter: 3 , chi= 561.804 , Lambda= 0.000221258
iter: 4 , chi= 4807.95 , Lambda= 7.37526e-05
iter: 5 , chi= 129277 , Lambda= 2.45842e-05
iter: 6 , chi= 1.50459e+07 , Lambda= 8.19474e-06
problem solve cost: 1.50635 ms
   makeHessian cost: 0.8488 ms

Compare MonoBA results after opt...
after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.555798 abs err: 0.33486
after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 1.20645 abs err: 0.972116
after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt -1.10419 abs err: 1.24653
after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 1.19364 abs err: 0.979324
after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.10191 abs err: 0.0287192
after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt -3.52752 abs err: 3.7189
after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.100494 abs err: 0.0663415
after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.992401 abs err: 0.790774
after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt -1.01339 abs err: 1.18135
after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt -0.596548 abs err: 0.815458
after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt -53.9447 abs err: 54.1504
after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt -0.312064 abs err: 0.43998
after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt -3.62888 abs err: 3.79678
after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt -6.68774 abs err: 6.90445
after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.431062 abs err: 0.251053
after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.930471 abs err: 0.703536
after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.888396 abs err: 0.730964
after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt -8.20445 abs err: 8.3869
after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 1.00337 abs err: 0.84767
after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 1.20093 abs err: 1.05447
Total abs error : 87.4006
------------ pose translation ----------------
translation after opt: 0 :-0.0131181  0.0860917  0.0356957 || gt: 0 0 0
translation after opt: 1 :-1.10562  4.01589  0.85115 || gt:  -1.0718        4 0.866025
translation after opt: 2 :-3.92653   6.7018 0.878972 || gt:       -4   6.9282 0.866025
---------- TEST Marg: before marg------------
     100     -100        0
    -100  136.111 -11.1111
       0 -11.1111  11.1111
---------- TEST Marg: 将变量移动到右下角------------
     100        0     -100
       0  11.1111 -11.1111
    -100 -11.1111  136.111
---------- TEST Marg: after marg------------
 26.5306 -8.16327
-8.16327  10.2041

```


## executing result with different prior weight.

#### prior weight set to 1
```
(base) brucesz@sbrucezhang-LB1:~/git/localization_slam/vio_assign/hw_course5_new/build$  ./app/testMonoBA  -add_prior -prior_w=1
0 order: 0
1 order: 6
2 order: 12

 ordered_landmark_vertices_ size : 20
iter: 0 , chi= 69.2375 , Lambda= 0.00597396
iter: 1 , chi= 89.9436 , Lambda= 2.03911
iter: 2 , chi= 231.047 , Lambda= 0.679704
iter: 3 , chi= 10266.6 , Lambda= 0.226568
iter: 4 , chi= 1.95124e+06 , Lambda= 0.0755227
iter: 5 , chi= 2.35098e+07 , Lambda= 0.201394
iter: 6 , chi= 1.29226e+09 , Lambda= 4.2964
iter: 7 , chi= 169093 , Lambda= 2.86427
problem solve cost: 1.80465 ms
   makeHessian cost: 0.849046 ms

Compare MonoBA results after opt...
after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.284269 abs err: 0.0633306
after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.412837 abs err: 0.1785
after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt -6.84796 abs err: 6.99029
after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.458999 abs err: 0.244684
after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.152955 abs err: 0.0223257
after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt -276.965 abs err: 277.156
after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.170392 abs err: 0.0035559
after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.243873 abs err: 0.0422455
after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt -6.29428 abs err: 6.46223
after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt -1.25889 abs err: 1.4778
after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt -2.61158e+06 abs err: 2.61158e+06
after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt -0.701925 abs err: 0.829841
after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt -223.835 abs err: 224.003
after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt -2250.64 abs err: 2250.86
after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.288476 abs err: 0.108467
after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.225292 abs err: 0.0016434
after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.364197 abs err: 0.206765
after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt -8906.11 abs err: 8906.29
after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.199896 abs err: 0.0441947
after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.29812 abs err: 0.151659
Total abs error : 2.62326e+06
------------ pose translation ----------------
translation after opt: 0 :-1.99495 -1.72453  4.26294 || gt: 0 0 0
translation after opt: 1 : 1.9217 2.81919 5.76297 || gt:  -1.0718        4 0.866025
translation after opt: 2 :-3.35809  8.23934 0.534243 || gt:       -4   6.9282 0.866025
---------- TEST Marg: before marg------------
     100     -100        0
    -100  136.111 -11.1111
       0 -11.1111  11.1111
---------- TEST Marg: 将变量移动到右下角------------
     100        0     -100
       0  11.1111 -11.1111
    -100 -11.1111  136.111
---------- TEST Marg: after marg------------
 26.5306 -8.16327
-8.16327  10.2041


```
we can see that opt value is closer to gt value, e.g point 19 , gt is 0.14646 , opt value is 0.29812
but the total abs err is bigger.
Maybe we can conclude that setting prior_w equal to 1 doesn't help in opt process?

#### prior weight set to 100
```
(base) brucesz@sbrucezhang-LB1:~/git/localization_slam/vio_assign/hw_course5_new/build$ make && ./app/testMonoBA  -add_prior -prior_w=100.0
[ 12%] Built target slam_course_frontend
[ 75%] Built target slam_course_backend
[ 87%] Built target testMonoBA
[100%] Built target testCurveFitting
0 order: 0
1 order: 6
2 order: 12

 ordered_landmark_vertices_ size : 20
iter: 0 , chi= 6394 , Lambda= 0.00597396
iter: 1 , chi= 6390.59 , Lambda= 0.254889
problem solve cost: 0.77055 ms
   makeHessian cost: 0.238871 ms

Compare MonoBA results after opt...
after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.183214 abs err: 0.0377239
after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.185014 abs err: 0.0493228
after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt 0.0494616 abs err: 0.0928742
after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.307018 abs err: 0.0927028
after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.104646 abs err: 0.025983
after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt 0.0662663 abs err: 0.125111
after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.108257 abs err: 0.0585789
after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.136667 abs err: 0.0649604
after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt 0.127469 abs err: 0.0404839
after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt 0.0712684 abs err: 0.147642
after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt 0.0467196 abs err: 0.159
after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt 0.0745746 abs err: 0.0533409
after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt 0.065886 abs err: 0.102018
after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt 0.13409 abs err: 0.0826216
after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.169025 abs err: 0.0109835
after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.147565 abs err: 0.0793701
after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.168413 abs err: 0.0109813
after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt 0.0733039 abs err: 0.109148
after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.14547 abs err: 0.0102311
after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.212665 abs err: 0.0662049
Total abs error : 1.41928
------------ pose translation ----------------
translation after opt: 0 :-0.00318679  0.00607378    0.011885 || gt: 0 0 0
translation after opt: 1 : -1.0678  3.97804 0.922773 || gt:  -1.0718        4 0.866025
translation after opt: 2 :-3.39484  6.17861  1.55148 || gt:       -4   6.9282 0.866025
---------- TEST Marg: before marg------------
     100     -100        0
    -100  136.111 -11.1111
       0 -11.1111  11.1111
---------- TEST Marg: 将变量移动到右下角------------
     100        0     -100
       0  11.1111 -11.1111
    -100 -11.1111  136.111
---------- TEST Marg: after marg------------
 26.5306 -8.16327
-8.16327  10.2041


```

conclusion: results seem to be better, for point 19, gt==0.14646, opt value== 0.212665
also the total abs error is reduced to 1.41

#### prior weight to 10000?
```
(base) brucesz@sbrucezhang-LB1:~/git/localization_slam/vio_assign/hw_course5_new/build$ make && ./app/testMonoBA  -add_prior -prior_w=10000.0
[ 12%] Built target slam_course_frontend
[ 75%] Built target slam_course_backend
[ 87%] Built target testMonoBA
[100%] Built target testCurveFitting
0 order: 0
1 order: 6
2 order: 12

 ordered_landmark_vertices_ size : 20
iter: 0 , chi= 638871 , Lambda= 0.10157
problem solve cost: 0.375981 ms
   makeHessian cost: 0.07859 ms

Compare MonoBA results after opt...
after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.227057 abs err: 0.00611848
after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.314411 abs err: 0.0800745
after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt 0.129703 abs err: 0.0126329
after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.278486 abs err: 0.0641714
after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.130064 abs err: 0.000565251
after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt 0.167501 abs err: 0.0238763
after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.165906 abs err: 0.000930183
after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.225581 abs err: 0.0239532
after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt 0.155846 abs err: 0.0121067
after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt 0.209697 abs err: 0.00921307
after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt 0.14315 abs err: 0.0625691
after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt 0.122109 abs err: 0.00580617
after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt 0.143334 abs err: 0.0245708
after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt 0.18526 abs err: 0.0314523
after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.184249 abs err: 0.00424004
after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.245716 abs err: 0.0187812
after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.176529 abs err: 0.0190969
after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt 0.14729 abs err: 0.0351616
after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.182258 abs err: 0.0265572
after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.240649 abs err: 0.0941887
Total abs error : 0.556066
------------ pose translation ----------------
translation after opt: 0 :0 0 0 || gt: 0 0 0
translation after opt: 1 : -1.0718        4 0.866025 || gt:  -1.0718        4 0.866025
translation after opt: 2 :      -4   6.9282 0.866025 || gt:       -4   6.9282 0.866025
---------- TEST Marg: before marg------------
     100     -100        0
    -100  136.111 -11.1111
       0 -11.1111  11.1111
---------- TEST Marg: 将变量移动到右下角------------
     100        0     -100
       0  11.1111 -11.1111
    -100 -11.1111  136.111
---------- TEST Marg: after marg------------
 26.5306 -8.16327
-8.16327  10.2041


```

in this case, we set the prior weight to 10000, the total abs error is reduced to 0.556066.
However, not every point's opt result is getting better, for point 19 , gt == 0.14646, 
now opt value is 0.240649 which is larger than that in prior_w == 100 case. 




