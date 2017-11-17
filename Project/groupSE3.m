function T = groupSE3(pose)
    gamma = pose(1);
    beta = pose(2);
    alpha = pose(3);
    x = pose(4);
    y = pose(5);
    z = pose(6);
    
	Rx = RotX(gamma);
	Ry = RotY(beta);
	Rz = RotZ(alpha);
	t = Tl(x, y, z);
% 	T = t.dot(Rz).dot(Ry).dot(Rx);
end

function Rx = RotX(gamma)
    s = sin(gamma);
    c = cos(gamma);
	Rx = [
	1,	0,	0,	0;
	0,	c,	-s,	0;
	0,	s,	c,	0;
	0,	0,	0,	1;
	];
end

function Ry = RotY(beta)
    s = sin(beta);
    c = cos(beta);
	Ry = [
	c,		0,	s,	0;
	0,		1,	0,	0;
	-s,	0,	c,	0;
	0,		0,	0,	1
	];
end

function Rz = RotZ(alpha)
    s = sin(alpha);
    c = cos(alpha);
	Rz = [
	c,	-s,	0,	0;
	s,	c,	0,	0;
	0,	0,	1,	0;
	0,	0,	0,	1;
	];
end

function T = Tl(x,y,z)
	T = [
	1,	0,	0,	x;
	0,	1,	0,	y;
	0,	0,	1,	z;
	0,	0,	0,	1;
	];





