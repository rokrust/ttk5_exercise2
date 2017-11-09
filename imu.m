function [a, w] = true_mes(t)
    a = [0.5*sin(t), 0.5*cos(t), 0];
    w = [0.2*cos(t), 0.2*cos(t), 0];
    
    if t > 50
        a = zeros(1, 3);
    end
end