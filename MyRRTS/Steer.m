function coord = Steer(qn, qr, d, stp)

if d <= stp
    coord = qr;
else
    coord(1) = qn(1) + (qr(1) - qn(1))*stp/d;
    coord(2) = qn(2) + (qr(2) - qn(2))*stp/d; 
end

end