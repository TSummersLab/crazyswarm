r_out = 1;
r_in = 0.5;
n_out = 10;
n_in = 5;
z_in = 0.5;
z_out = 1.0;
N = 16;

q_in = zeros(n_in,3);
for i = 1 : n_in
    ang = 360/n_in * i;
    q_in(i,1) = r_in * cosd(ang);
    q_in(i,2) = r_in * sind(ang);
    q_in(i,3) = z_in;
end

q_out = zeros(n_out,3);
for i = 1 : n_out
    ang = 360/n_out * i;
    q_out(i,1) = r_out * cosd(ang);
    q_out(i,2) = r_out * sind(ang);
    q_out(i,3) = z_out;
end

    
qDes = [0., 0., 0.];
qDes = [qDes ; q_in;  q_out] * 1.0;

scatter3(qDes(:,1), qDes(:,2), qDes(:,3))
hold on

adjvec = [1 1 1 1 1  2 3 4 5 6  7 8  9 10 11 12 13 14 15 16  2  2 3 3  4  4  5  5  6  6   1  1; 
          2 3 4 5 6  3 4 5 6 2  8 9 10 11 12 13 14 15 16  7  7 16 8 9 10 11 12 13 14 15  11 16];

adj = zeros(N,N);      
for i = 1 : size(adjvec,2)
    id1 = adjvec(1,i);
    id2 = adjvec(2,i);
    adj(id1, id2) = 1;    
end
adj = adj + adj';

adj(:,1) = 1;
adj(1,:) = 1;
adj(1,1) = 0;




% 
% adj = [ ...
% 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   ; ...
% 1   0   1   0   0   1   1   0   0   0   0   0   0   0   0   1   ; ...
% 1   1   0   1   0   0   0   1   1   0   0   0   0   0   0   0   ; ...
% 1   0   1   0   1   0   0   0   0   1   1   0   0   0   0   0   ; ...
% 1   0   0   1   0   1   0   0   0   0   0   1   1   0   0   0   ; ...
% 1   1   0   0   1   0   0   0   0   0   0   0   0   1   1   0   ; ...
% 1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   ; ...
% 1   0   1   0   0   0   1   0   1   0   0   0   0   0   0   0   ;...
% 1   0   1   0   0   0   0   1   0   1   0   0   0   0   0   0   ;...
% 1   0   0   1   0   0   0   0   1   0   1   0   0   0   0   0   ;...
% 1   0   0   1   0   0   0   0   0   1   0   1   0   0   0   0   ;...
% 1   0   0   0   1   0   0   0   0   0   1   0   1   0   0   0   ;...
% 1   0   0   0   1   0   0   0   0   0   0   1   0   1   0   0   ;...
% 1   0   0   0   0   1   0   0   0   0   0   0   1   0   1   0   ;...
% 1   0   0   0   0   1   0   0   0   0   0   0   0   1   0   1   ;...
% 1   1   0   0   0   0   1   0   0   0   0   0   0   0   1   0  ];

N = size(adj,1);
X = qDes(:,1);
Y = qDes(:,2);
Z = qDes(:,3);

for m = 1 : N
    for n = 1 : N
        if adj(m,n)
        plot3([X(m),X(n)],[Y(m),Y(n)], ...
             [Z(m),Z(n)], 'LineWidth',2, ...
             'Color', [0.4 0.4 0.4]);
        end
    end
end  
axis equal

n = N;
qDest = qDes';
q = qDest(:);
Df = zeros(n,n);
for i = 1 : n
    for j = i+1 : n
        Df(i,j) = norm(q(3*i-2:3*i)-q(3*j-2:3*j), 2);
    end
end
Df = Df + Df'


[Ar, Kd] = FindGains3D_Ver1_0(qDes, adj);
sort(eig(Ar))