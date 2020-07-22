a = stairs(240, 200, 9.8065);
R = [150, 150, 150];
robert = rockerbogie(481, 194, 225, 225, 170, 94, R);
Lr = [400, 600];
Ll = [180, 600];
lr = [155, 300];
ll = [155, 300];
db = [155, 200];
hb = [25, 200];
robert = robert.Optimize(1, a, Lr, Ll, lr, ll, db, hb, R, 3, 15);
robert.CompareCG(a, 1000)