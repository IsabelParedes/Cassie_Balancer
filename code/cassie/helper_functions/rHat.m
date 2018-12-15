function r = rHat(xyz)

x = xyz(1);
y = xyz(2);
z = xyz(3);

r = [0 -z  y;
     z  0 -x;
    -y  x  0];