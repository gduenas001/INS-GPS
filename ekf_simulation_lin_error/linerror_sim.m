clear; format short e; clc; close all;
%dbstop if error

addpath('function');
addpath('class');

% parameters
SIM_SWITCH = 1;
Ireq = 2*1e-9;
sample_num = 1e+6;

sim_str = {'Single variable','Multivariant','Simple robot simulation'};
prompt = 'Select the type of simulation';
[sim_select,ok] = listdlg('PromptString',prompt,'SelectionMode','single','ListString',sim_str);

% single variable simulation block
% 1: simulate with cosx
% 2: simulate with x^2
if sim_select == 1
    % parameters
    % for single variable
    mean_x = 0.5;
    cov_x = 0.25;

    param_single = ParamClass(mean_x, cov_x, Ireq, sample_num, SIM_SWITCH);

    sim_single1 = LinErrClass(param_single);
    sim_single2 = LinErrClass(param_single);

    sim_single1.single_cov(1);
    sim_single2.single_cov(2);

    sim_single1.single_Ru_cov(1);
    sim_single2.single_Ru_cov(2);

    sim_single1.single_montecarlo_cov(1);
    sim_single2.single_montecarlo_cov(2);

    disp("Without overbounding");
    disp("cosx: " + num2str(sim_single1.cov_y^2));
    disp("x^2: " + num2str(sim_single2.cov_y^2));

    disp("Over bounded covariances");
    disp("cosx: " + num2str(sim_single1.cov_y_ub^2));
    disp("x^2: " + num2str(sim_single2.cov_y_ub^2));

    disp("Montecarlo approach");
    disp("cosx: " + num2str(sim_single1.sample_cov_y));
    disp("x^2: " + num2str(sim_single2.sample_cov_y));

    sim_single1.histogram_y(1);
    sim_single1.cdf_plot(2);

    sim_single2.histogram_y(3);
    sim_single2.cdf_plot(4);
elseif sim_select == 2
  % parameters
  % for multivariant
  mean_x = [0.5 1];
  cov_x = [0.0025 0; 0 0.0025];

  param_multi = ParamClass(mean_x, cov_x, Ireq, sample_num, SIM_SWITCH);

  sim_multi1 = LinErrClass(param_multi);
  %sim_multi2 = LinErrClass(param_multi);

  sim_multi1.multi_cov(1);
  %sim_multi2.multi_cov(2);

  sim_multi1.multi_Ru_cov(1);
  %sim_multi2.multi_Ru_cov(2);

  sim_multi1.multi_montecarlo_cov(1);
  %sim_multi2.multi_montecarlo_cov(2);

  disp("Without overbounding");
  disp("cosx: " + num2str(sim_multi1.cov_y^2));
  %disp("x^3: " + num2str(sim_multi2.cov_y^2));

  disp("Over bounded covariances with Plan A");
  disp("cosx: " + num2str(sim_multi1.cov_y_ub^2));
  %disp("x^3: " + num2str(sim_multi2.cov_y_ub^2));

  disp("Over bounded covariances with Plan B");
  disp("cosx: " + num2str(sim_multi1.cov_y_ub_b^2));
  %disp("x^3: " + num2str(sim_multi2.cov_y_ub_b^2));

  disp("Montecarlo approach");
  disp("cosx: " + num2str(sim_multi1.sample_cov_y));
  %disp("x^3: " + num2str(sim_multi2.sample_cov_y));

  sim_multi1.histogram_y(1);
  sim_multi1.cdf_plot(2);

  %sim_multi2.histogram_y(3);
  %sim_multi2.cdf_plot(4);

elseif sim_select == 3
  mean_x = [1,1,0.5];
  cov_x = [0.3,0,0;0,0.2,0;0,0,0.001];
  v = 1;
  w = 1;
  dt = 0.01;
  lm_position = [2 2; 3 1; 1 3];

  param_sr = ParamClass(mean_x, cov_x, Ireq, sample_num,lm_position, SIM_SWITCH);
  sim_sr = LinErrClass(param_sr);

  sim_sr.sr_sim(v,w,dt);
  %sim_sr.sr_sim_plot_I(1);


  % Displaying Innovation part
  disp("Innovation");
  disp("Without overbounding");
  disp("Sigma:");
  disp(sim_sr.cov_y);

  disp("Over bounded");
  disp("Sigma:");
  disp(sim_sr.cov_y_ub);

  disp("Montecarlo approach");
  disp("Sigma:");
  disp(sim_sr.sample_cov_y);
  disp("===================");
  % Displaying Observation part
  disp("Observation");
  disp("Without overbounding");
  for i = 1:size(sim_sr.lm_position,1)
    disp("Covariance matrix of landmark "+num2str(i));
    disp(sim_sr.cov_z(2*(i-1)+1:2*i,2*(i-1)+1:2*i));
  end
  disp("===================");
  disp("Over bounded, Plan A");
  for i = 1:size(sim_sr.lm_position,1)
    disp("Covariance matrix of landmark "+num2str(i));
    disp(sim_sr.cov_z_ub(2*(i-1)+1:2*i,2*(i-1)+1:2*i));
  end
  disp("===================");
  disp("Over bounded, Plan B");
  for i = 1:size(sim_sr.lm_position,1)
    disp("Covariance matrix of landmark "+num2str(i));
    disp(sim_sr.cov_z_ub_b(2*(i-1)+1:2*i,2*(i-1)+1:2*i));
  end
  disp("===================");
  disp("Montecarlo approach");
  for i = 1:size(sim_sr.lm_position,1)
    disp("Covariance matrix of landmark "+num2str(i));
    disp(sim_sr.sample_cov_z(2*(i-1)+1:2*i,2*(i-1)+1:2*i));
  end

end

% Decleene upper bound calculation ->make lookup table for n dof
% consider bias
