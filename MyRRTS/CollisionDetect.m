function chk = CollisionDetect(qnear,qnew,xobs,yobs)

% - this function checks for collision "intersection" between line made from near and new
% points and every edge of the obstacle

chk = 0; % - if this value changed to 1 this means there was intersection
         % if not then intersection didn't happen

for i = 1 : (length(xobs)-1)
    
    dt1=det([1,1,1;qnear(1),qnew(1),xobs(i);qnear(2),qnew(2),yobs(i)])*det([1,1,1;qnear(1),qnew(1),xobs(i+1);qnear(2),qnew(2),yobs(i+1)]);
    dt2=det([1,1,1;qnear(1),xobs(i),xobs(i+1);qnear(2),yobs(i),yobs(i+1)])*det([1,1,1;qnew(1),xobs(i),xobs(i+1);qnew(2),yobs(i),yobs(i+1)]);
    
    if(dt1<=0 & dt2<=0)
      chk = 1; % there is intersection
      break
    end
      
end