function tgt=limitRange(tgt)
if tgt(1)>0.9*pi
    tgt(1) = 0.9*pi;
end
if tgt(1)<-0.9*pi
    tgt(1)= -0.9*pi;
end

if tgt(2)>4
    tgt(2) = 4;
end
if tgt(2)<-0.5
    tgt(2)= -0.5;
end