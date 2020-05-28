a = stairs(240, 200, 9.8065);
hold on;
a.draw_stairs(1000);

robert = rockerbogie(481, 194, 225, 225, 170, 94, 150, 150, 150);
%randomly assign an x y theta value for now
xrand = 300;
yrand = 200;
th = 0;
robert = robert.DetectPos(1, a, xrand, yrand, th);
robert.draw();
xlim([-1000 1000])
ylim([0 2000])