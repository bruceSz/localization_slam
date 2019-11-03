## code execution result

#### default execution  (same  lambda update stragegy 3 in paper)
 
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

 * 4 plot mu change with iteration, see mu_change.pdf 


#### second execution (lambda update stategy 1 in paper, refer  problem_1.cc)
   * 1 formula:
      **  ifρi(h)> 4:p←p+h;λi+1= max[λi/L↓,10−7]
      ** otherwise:λi+1= min [λiL↑,107]
      ** with Marquardt's update relationship:
      ** [JTW J+λdiag(JTW J)]hlm=JTW(y−ˆy)

   * 2 change desciption
      ** init val of lambda change
      ** rho computaion change.
      ** lambda update logic.

   * 3 execution result:
      <code>Test CurveFitting start...
iter: 0 , chi= 36048.3 , Lambda= 0.01
iter: 1 , chi= 34603.6 , Lambda= 16.2678
iter: 2 , chi= 5202.26 , Lambda= 1.80753
iter: 3 , chi= 737.707 , Lambda= 0.200837
iter: 4 , chi= 355.065 , Lambda= 0.0223152
iter: 5 , chi= 141.356 , Lambda= 0.00247947
iter: 6 , chi= 100.515 , Lambda= 0.000275496
iter: 7 , chi= 92.175 , Lambda= 3.06107e-05
iter: 8 , chi= 91.3988 , Lambda= 3.40119e-06
iter: 9 , chi= 91.3959 , Lambda= 3.7791e-07
iter: 10 , chi= 91.3959 , Lambda= 1e-07
Stop earlier. squared norm: 3.146816312751e-12 False count: 0
problem solve cost: 0.559499 ms
   makeHessian cost: 0.373656 ms
-------After optimization, we got these parameters :
0.941840288068901  2.09467469546203 0.965536299943316
-------ground truth: 
1.0,  2.0,  1.0


      </code>

#### third execution (lambda update strategy 2 in paper, refer problem_2.cc)
   * 1 formula:
      **  introduce alpha : α=((JTW(y−ˆy(p)))Th)/((χ2(p+h)−χ2(p))/2 + 2(JTW(y−ˆy(p)))Th)
      ** delta_x update has to consider this alpha
      ** lambda update consider alpha too.
   * 2 change description
      ** 