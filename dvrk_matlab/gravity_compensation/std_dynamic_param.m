function [ std_var_beta, rel_std_var_beta ] = std_dynamic_param( R2, T2, dynamic_param_vec )
% Compute the standard deviation and relative std of estimated dynamic parameters
    if size(R2,1)~=size(T2,1)
        error(sprintf('The rows of regressor matrix = %d and torque matrix = %d are not equal',size(R2,1),size(T2,1)));
    end

    if size(dynamic_param_vec,2)~=1
        error(sprintf('dynamic_param_vec should be a vector'));
    end

    if size(T2,2)~=1
        error(sprintf('T2 should be a vector'));
    end

    if size(R2,2)~=size(dynamic_param_vec,1)
        error(sprintf('The columns of regressor matrix = %d and rows of dynamic_param_vec = %d are not equal',size(R2,2),size(dynamic_param_vec,1)));
    end


    var_e = (norm(T2 - R2*dynamic_param_vec)^2)/(size(R2,1)-size(dynamic_param_vec,1));

    std_var_beta = sqrt(diag(var_e* inv((R2.'*R2)) ));

    rel_std_var_beta = std_var_beta*100/norm(std_var_beta);

end

