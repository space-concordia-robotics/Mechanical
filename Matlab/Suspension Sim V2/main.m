a = stairs(240, 200, 9.8065);
hold on;
a.draw_stairs(2000);
R = [150, 150, 150];
robert = rockerbogie(481, 194, 225, 225, 170, 94, R);
%arbitrarily assign an x y theta value for now
xrand1 = 1500;
yrand1 = 1200;
xrand2 = 500;
yrand2 = 400;
th = 0;
robert = robert.DetectPos(1, a, xrand1, yrand1, th, 1);
robert = robert.DetectPos(1, a, xrand2, yrand2, th, 2);
robert.draw(1);
robert.draw(2);
xlim([-1000 2000])
ylim([0 3000])