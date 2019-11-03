1;
## Function : write_to_csv()
## ----------------------------------------------------
## Input:   file_id, eqbm_pts, jacobians, eigen_values, stability
## Output:  None
## Purpose: This function takes eqbm_pts, jacobians, eigen_values and stability
##          and writes them to a filename Solutions.csv
function write_to_csv(file_id, x1_dot, x2_dot, eqbm_pts, jacobians, eigen_values, stability)
  filename = "Solutions.csv";
  file_id = fopen(filename, "w");
  
  fputs (file_id, char(x1_dot));
  fputs (file_id, ",");
  fputs (file_id, char(x2_dot));
  fputs (file_id, ",");
  fputs (file_id, "Solutions,");
  
  num_rows = rows(eqbm_pts);
  num_cols = columns(eqbm_pts);
  for a=1:num_rows
    for b=1:num_cols
      x_1 = double(eqbm_pts{a, b}.x1);
      x_2 = double(eqbm_pts{a, b}.x2);
      soln = strcat(num2str(x_1),"_",num2str(x_2),',');
      fputs(file_id, soln);
    endfor
  endfor
  
  fputs (file_id, "Jacobians,");
  for i=1:length(jacobians)
    J_i = jacobians{i};
    J_matrix = strcat(num2str(J_i(1,1)), "_", num2str(J_i(1,2)), ";", num2str(J_i(2,1)), "_", num2str(J_i(2,2)), ",");
    fputs(file_id, J_matrix);
  endfor
  
  fputs (file_id, "Eigen Values,");
  for k=1:length(eigen_values)
    eig_val = eigen_values{k};
    e_matrix = strcat(num2str(eig_val(1)),"_",num2str(eig_val(2)),",");
    fputs(file_id, e_matrix);
  endfor
  
  fputs (file_id, "Stability,");
  for k=1:length(stability)
    stab = stability{k};
    s_matrix = strcat(stab,",");
    fputs(file_id, s_matrix);
  endfor
  fputs(file_id, "\n");
  #fclose(file_id);
endfunction


pkg load symbolic      # Load the octave symbolic library
syms x1 x2             # Define symbolic variables x1 and x1
filename = "Solutions.csv";
file_id = fopen(filename, "w");

fprintf("1. Check the stability of system of equations:\n\n");
fprintf("   x1_dot = -2*x1 + x1*x2\n");
fprintf("   x2_dot = 2*x1*x1 - x2\n");
x1_dot = -2*x1 + x1*x2;
x2_dot = 2*x1*x1 - x2;

try
  [equilibrium_points jacobians eigen_values stability] = main_function(x1_dot, x2_dot);
catch
  printf("Error in evaluating example 1\n")
end_try_catch

try
  write_to_csv(file_id, x1_dot, x2_dot, equilibrium_points, jacobians, eigen_values, stability);
catch
  printf("Error in writing example 1 to file\n")
end_try_catch

fprintf("2. Check the stability of system of equations:\n\n");
fprintf("   x1_dot = x1*x2 - x2\n");
fprintf("   x2_dot = 2*x1-x2^2\n");
x1_dot = x1*x2 - x2;
x2_dot = 2*x1-x2^2;

try
  [equilibrium_points jacobians eigen_values stability] = main_function(x1_dot, x2_dot);
catch
  printf("Error in evaluating example 2\n")
end_try_catch

try
  write_to_csv(file_id, x1_dot, x2_dot, equilibrium_points, jacobians, eigen_values, stability);
catch
  printf("Error in writing example 2 to file\n")
end_try_catch


fprintf("3. Check the stability of system of equations:\n\n");
fprintf("   x1_dot = x1^2*x2 + 2*x1\n");
fprintf("   x2_dot = x2 +x1 - 1\n");
x1_dot = x1^2*x2 + 2*x1;
x2_dot = x2 +x1 - 1;

try
  [equilibrium_points jacobians eigen_values stability] = main_function(x1_dot, x2_dot);
catch
  printf("Error in evaluating example 3\n")
end_try_catch

try
  write_to_csv(file_id, x1_dot, x2_dot, equilibrium_points, jacobians, eigen_values, stability);
catch
  printf("Error in writing example 3 to file\n")
end_try_catch

fclose(file_id);
