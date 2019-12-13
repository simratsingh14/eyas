clc;
close all;
clear all;

global A = csvread('csv_matter.csv');  #do not change this line

################################################
#######Declare your global variables here#######

global lpx = 0;                        # Low Pass filtered AccelX
global lpy = 0;                        # Low Pass filtered AccelY
global lpz = 0;                        # Low Pass filtered AccelZ

global B=0;                            
                                         
global hpx = 0;                        # High Pass filtered GyroX
global hpy = 0;                        # High Pass filtered GyroY
global hpz = 0;                        # High Pass filtered GyroZ

global pitch=0;                        # Final Pitch
global roll=0;                         # Final Roll

##################################################

function read_accel(axl,axh,ayl,ayh,azl,azh) 
    
#################################################
####### Write a code here to combine the ########
#### HIGH and LOW values from ACCELEROMETER #####

  accel_sf = 16384;                      #scaling factor for accelerometer
  f_cut = 5;                             #cut off frequency
  
  ax = bitshift(axh,8)+axl;   
  ay = bitshift(ayh,8)+ayl;
  az = bitshift(azh,8)+azl;

 #conversion to signed  
 for i = 1:8000
 
    if (ax(i,1)) > 32767
      ax(i,1) = (ax(i,1) - 65536);
    endif

    if (ay(i,1)) > 32767
      ay(i,1) = (ay(i,1) - 65536);
    endif
  
    if (az(i,1)) > 32767
      az(i,1) = (az(i,1) - 65536);
    endif

 endfor
  ax = ax/accel_sf;
  ay = ay/accel_sf;
  az = az/accel_sf;
  
#################################################
# Called function lowpassfilter(ax,ay,az,f_cut) here #
  lowpassfilter(ax, ay,az, f_cut);
####################################################
  
endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  
#################################################
####### Write a code here to combine the ########
###### HIGH and LOW values from GYROSCOPE #######

  gyro_sf = 131;                        #scaling factor for gyroscope
  f_cut = 5;  
  
  gx = bitshift(gxh,8)+gxl;  
  gy = bitshift(gyh,8)+gyl;
  gz = bitshift(gzh,8)+gzl;
  
  for i = 1:8000
  
    if (gx(i,1)) > 32767
      gx(i,1) = (gx(i,1) - 65536);
    endif
  
    if (gy(i,1)) > 32767
      gy(i,1) = (gy(i,1) - 65536);
    endif
  
    if (gz(i,1)) > 32767
      gz(i,1) = (gz(i,1) - 65536);  
    endif
    
  endfor

  gx = gx/gyro_sf;
  gy = gy/gyro_sf;
  gz = gz/gyro_sf;
  
################################################## 
# Call function highpassfilter(ax,ay,az,f_cut) here #
   highpassfilter(gx,gy,gz,f_cut);
##################################################
   endfunction


function lowpassfilter(ax,ay,az,f_cut)
  
  global lpx;
  global lpy;
  global lpz;
  dT = 0.01;                                   # time in seconds
  Tau= double(1/(2*pi*f_cut));                 # Time constant
  alpha = double(Tau/(Tau+dT));                #do not change this line
  
################################################
##############Write your code here##############

  lpx(1,1) = (1-alpha)*ax(1);        
  lpy(1,1) = (1-alpha)*ay(1);
  lpz(1,1) = (1-alpha)*az(1);

  for n = 2:8000
      lpx(n,1) = (1-alpha)*ax(n,:) + alpha*lpx(n-1,1);
      lpy(n,1) = (1-alpha)*ay(n,:) + alpha*lpy(n-1,1);
      lpz(n,1) = (1-alpha)*az(n,:) + alpha*lpz(n-1,1);
  endfor
  
################################################

endfunction

function highpassfilter(gx,gy,gz,f_cut)
  
  global hpx;
  global hpy;
  global hpz;
  
  dT = 0.01;                                   #time in seconds
  Tau= double(1/(2*pi*f_cut));
  alpha = double(Tau/(Tau+dT));                #do not change this line
  
################################################
##############Write your code here##############

  hpx(1,1) = (1-alpha)*gx(1,1);
  hpy(1,1) = (1-alpha)*gy(1,1);
  hpz(1,1) = (1-alpha)*gz(1,1);

  for n = 2:8000
       hpx(n,1) = (1-alpha)*(hpx(n-1,1) + (gx(n,1)-gx(n-1,1)));
       hpy(n,1) = (1-alpha)*(hpy(n-1,1) + (gy(n,1)-gy(n-1,1)));
       hpz(n,1) = (1-alpha)*(hpz(n-1,1) + (gz(n,1)-gz(n-1,1)));
  endfor
  
 ################################################ 
endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)

  global pitch;
  comp_alpha = 0.03;
  dT = 0.01;
  
##############################################
####### Write a code here to calculate  ######
####### PITCH using complementry filter ######  

  accel_pitch = atand(ay/abs(az));
  pitch  = (1 - comp_alpha)*(pitch - gx*dT) + comp_alpha*accel_pitch;
  
##############################################
endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)
 
   global roll;
   comp_alpha = 0.03;
   dT = 0.01;
##############################################
####### Write a code here to calculate  ######
####### ROLL using complementry filter #######  

   accel_roll = atand(ax/abs(az));
   roll = (1 - comp_alpha)*(roll - gy*dT) + comp_alpha*accel_roll;

##############################################
endfunction 

function execute_code()
  global A;
  global lpx;
  global lpy;
  global lpz;
  global hpx;
  global hpy;
  global hpz;
  global B;
  global pitch;
  global roll;
  
  axh = A(:,1);                        #ACCEL_XOUT_H
  axl = A(:,2);                        #ACCEL_XOUT_L
  ayh = A(:,3);                        #ACCEL_YOUT_H
  ayl = A(:,4);                        #ACCEL_YOUT_L
  azh = A(:,5);                        #ACCEL_ZOUT_H
  azl = A(:,6);                        #ACCEL_ZOUT_L
  
  gxh = A(:,7 );                       #GYRO_XOUT_H
  gxl = A(:,8 );                       #GYRO_XOUT_L
  gyh = A(:,9 );                       #GYRO_YOUT_H 
  gyl = A(:,10);                       #GYRO_YOUT_L
  gzh = A(:,11);                       #GYRO_ZOUT_H
  gzl = A(:,12);                       #GYRO_ZOUT_L
  
  read_accel(axl,axh,ayl,ayh,azl,azh);
  read_gyro(gxl,gxh,gyl,gyh,gzl,gzh);
  
  for n = 1:rows(A)                 #do not change this line        
    
    comp_filter_pitch(lpx(n,1),lpy(n,1),lpz(n,1),hpx(n,1),hpy(n,1),hpz(n,1));
    comp_filter_roll(lpx(n,1),lpy(n,1),lpz(n,1),hpx(n,1),hpy(n,1),hpz(n,1));
    
    #pitch and roll values stored in B
    B(n,1) = pitch;
    B(n,2) = roll; 
    
  endfor
  csvwrite('output_data.csv',B);    #do not change this line   
endfunction

execute_code                        #do not change this line
