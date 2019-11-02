## code execution result

#### default parameter 

* 1  formular: y = exp(x*x*a + x*b +c)

* 2  execution result:
<code>
brucesz@brucesz-N9x0TD-TF:~/work/brucesz/git/localization_slam/vio_assign/curve_build$ ./app/testCurveFitting 
Test CurveFitting start...
iter: 0 , chi= 36048.3 , Lambda= 0.001
iter: 1 , chi= 30015.5 , Lambda= 699.051
iter: 2 , chi= 13421.2 , Lambda= 1864.14
iter: 3 , chi= 7273.96 , Lambda= 1242.76
iter: 4 , chi= 269.255 , Lambda= 414.252
iter: 5 , chi= 105.473 , Lambda= 138.084
iter: 6 , chi= 100.845 , Lambda= 46.028
iter: 7 , chi= 95.9439 , Lambda= 15.3427
iter: 8 , chi= 92.3017 , Lambda= 5.11423
iter: 9 , chi= 91.442 , Lambda= 1.70474
iter: 10 , chi= 91.3963 , Lambda= 0.568247
iter: 11 , chi= 91.3959 , Lambda= 0.378832
iter: 12 , chi= 91.3959 , Lambda= 0.252554
Stop earlier. squared norm: 2.67381754874209e-11 False count: 0
problem solve cost: 3.220775 ms
   makeHessian cost: 2.070782 ms
-------After optimization, we got these parameters :
0.941842204598225  2.09467192140342 0.965537230086095
-------ground truth: 
1.0,  2.0,  1.0
</code>

 * 3 analysis.
 * 4 plot mu change with iteration, see mu_change.pdf 


