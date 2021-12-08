function y=forward_unic(x)
    global M

    % ReLU 1:
    x=M{1}*x+M{2};
    x(x<0)=0;
    
    % ReLU 2:
    x=M{3}*x+M{4};
    x(x<0)=0;

    % ReLU 3:
    x=M{5}*x+M{6};
    x(x<0)=0;

    % linear output:
    y=M{7}*x+M{8};
end