# code execution result

### default execution  (same  lambda update stragegy 3 in paper)
 
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


### second execution (lambda update stategy 1 in paper, refer  problem_1.cc)
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

### third execution (lambda update strategy 2 in paper, refer problem_2.cc)
   * 1 formula:
      **  introduce alpha : α=((JTW(y−ˆy(p)))Th)/((χ2(p+h)−χ2(p))/2 + 2(JTW(y−ˆy(p)))Th)
      ** delta_x update has to consider this alpha
      ** lambda update consider alpha too.
   * 2 change description
      ** mainly in IsGoodStepInLM function.
   * 3 execution result:
     <code>Test CurveFitting start...
iter: 0 , chi= 36048.3 , Lambda= 0.001
alpha of delta x: 0.1
iter: 1 , chi= 9294.65 , Lambda= 0.000909091
alpha of delta x: 0.42032
iter: 2 , chi= 3097.71 , Lambda= 0.000640061
alpha of delta x: 0.66358
iter: 3 , chi= 436.783 , Lambda= 0.000384749
alpha of delta x: 0.665828
iter: 4 , chi= 132.226 , Lambda= 0.000230966
alpha of delta x: 0.666107
iter: 5 , chi= 96.0453 , Lambda= 0.000138626
alpha of delta x: 0.666174
iter: 6 , chi= 91.9154 , Lambda= 8.32001e-05
alpha of delta x: 0.666197
iter: 7 , chi= 91.4535 , Lambda= 4.99341e-05
alpha of delta x: 0.666211
iter: 8 , chi= 91.4022 , Lambda= 2.99687e-05
alpha of delta x: 0.666224
iter: 9 , chi= 91.3966 , Lambda= 1.7986e-05
alpha of delta x: 0.666236
iter: 10 , chi= 91.3959 , Lambda= 1.07944e-05
alpha of delta x: 0.666248
rho not good, new lambda: 6.2847e-05
alpha of delta x: 0.666249
rho not good, new lambda: 0.0001149
alpha of delta x: 0.666249
rho not good, new lambda: 0.000166952
alpha of delta x: 0.666249
rho not good, new lambda: 0.000219005
alpha of delta x: 0.666249
rho not good, new lambda: 0.000271057
alpha of delta x: 0.66625
rho not good, new lambda: 0.00032311
alpha of delta x: 0.66625
rho not good, new lambda: 0.000375162
alpha of delta x: 0.66625
rho not good, new lambda: 0.000427214
alpha of delta x: 0.66625
rho not good, new lambda: 0.000479267
alpha of delta x: 0.666251
rho not good, new lambda: 0.000531319
alpha of delta x: 0.666251
rho not good, new lambda: 0.000583371
Stop earlier. squared norm: 1.67961362254066e-06 False count: 11
problem solve cost: 3.689344 ms stop threshold: 0.0360483445707366currentChi_: 9.56012251888294
   makeHessian cost: 1.623731 ms
-------After optimization, we got these parameters :
0.942663542790209  2.09369918914733 0.965767761753611
-------ground truth: 
1.0,  2.0,  1.0

   </code> 

   ### change curve function to 
   * 1 formula:
      y = y = a*x *x + b*x+c
   * 2 change description:
     * 1 entry function main2 
     * 2 new CurveFittingEdge2 class for jacobians and new CurveFittingVertex2 class
   * 3 execution result (with default lambda update logic):
  
   <code>Test CurveFitting start...
iter: 0 , chi= 3.31191e+06 , Lambda= 19.95
After update delta, chi: 97388.7
 Old chi: 3.31191e+06 new tempChi: 97388.7 rho: 1 quare_norm of delta:  5.79731 lambda: 19.95
iter: 1 , chi= 97388.7 , Lambda= 6.65001
After update delta, chi: 97388
 Old chi: 97388.7 new tempChi: 97388 rho: 0.998584 quare_norm of delta:  0.00032995 lambda: 6.65001
iter: 2 , chi= 97388 , Lambda= 2.21667
After update delta, chi: 97388
 Old chi: 97388 new tempChi: 97388 rho: 0.022823 quare_norm of delta:  1.82289e-07 lambda: 2.21667
iter: 3 , chi= 97388 , Lambda= 1.47778
Stop earlier. squared norm: 9.42310463542048e-11 False count: 0
problem solve cost: 1.781528 ms
   makeHessian cost: 1.187802 ms
-------After optimization, we got these parameters :
0.995892399275448  2.06283090098366 0.788217740550153
-------ground truth: 
1.0,  2.0,  1.0

    </code>