function u = true_mes(t)
    a = exp(-0.0005*t)*sin(0.05*t);
    w = 0.2*cos(0.05*t);
    
    u = [a; w];
end