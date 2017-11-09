function u = true_mes(t)
    a = 0.5*sin(t);
    w = 0.2*cos(t);
    
    if t > 50
        a = 0;
    end
    
    u = [a; w]
end