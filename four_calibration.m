pw1 = [524.14 -486.13 102.3]' / 1000.0
pw2 = [443.52 -405.17 29.21]' / 1000.0
pw3 = [602.66 -404.69 28.39]' / 1000.0
pw4 = [445.31 -564.91 26.3]' / 1000.0

cMo = [-0.049083  -0.997588  0.0490735  0.0669251
 -0.993778  0.0536957  0.0975816 -0.0288339
-0.0999813 -0.0439785  -0.994017   0.783516
         0          0          0          1
];

cMo1 = cMo*[1 0 0 0;0 1 0 0;0 0 1 0.0748; 0 0 0 1];
cMo2 = cMo*[1 0 0 -0.08;0 1 0 0.08;0 0 1 0; 0 0 0 1];
cMo3 = cMo*[1 0 0 0.08;0 1 0 0.08;0 0 1 0; 0 0 0 1];
cMo4 = cMo*[1 0 0 -0.08;0 1 0 -0.08;0 0 1 0; 0 0 0 1];

pc1 = cMo1(1:3, 4)
pc2 = cMo2(1:3, 4)
pc3 = cMo3(1:3, 4)
pc4 = cMo4(1:3, 4)

Pw = [pw1-pw2, pw1-pw3, pw1-pw4];
Pc = [pc1-pc2, pc1-pc3, pc1-pc4];

R = Pw * inv(Pc);
wMc = [R, -R*pc1+pw1;0 0 0 1]