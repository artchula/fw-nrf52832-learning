function [output] = DCRemover(input)
    alpha = 0.95;
    persistent dcw;
    
    olddcw = dcw;
    
    if nargin == 0
        dcw = 0;
    else
    
    dcw = input + alpha * dcw;
    
    output = dcw - olddcw;
    
    end
end