p3d2navsub = rossubscriber('/cora/sensors/p3d_nav');
while true
   msg = receive(p3d2navsub,10);
   showdetails(msg);
end