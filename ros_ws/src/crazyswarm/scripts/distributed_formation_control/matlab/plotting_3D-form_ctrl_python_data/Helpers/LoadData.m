function [qInit, qDes, adj, Dd, n] = LoadData(formation)

% Define desired formation parameters:
switch formation
    case 'Tetrahedron' 
        % Tetrahedron coordinates
        xyz = [ sqrt(8/9),  0        ,   -1/3;
               -sqrt(2/9),  sqrt(2/3),   -1/3;
               -sqrt(2/9), -sqrt(2/3),   -1/3;
                0        ,  0        ,    1  ].' .*  0.6;

        % Graph adjacency matrix for tetrahedron
        adj = [ 0     1     1     1;
                1     0     1     1;
                1     1     0     1;
                1     1     1     0];
            
    case 'Square-base-pyramid'
        % Square base pyramid / pyramid coordinates
        xyz = [ -0.5,  0.0, 0.0; ...
                -0.5, -1.0, 0.0; ...
                 0.5, -1.0, 0.0; ...
                +0.5,  0.0, 0.0; ...
                 0.0, -0.5, 1.0].' .* 1.0;
             
        % Graph adjacency matrix for pyramid
        adj = ones(5) - eye(5);
    
    case 'Pentagon-base-pyramid'
        % Pentagon base pyramid cordinates
        xyz = [0.9511, 0.3090, 0.0;...
                0.5878, -0.8090, 0.0;...
                -0.5878, -0.8090, 0.0;...
                0.0, 0.0, 1.0;...
                -0.9511, 0.3090, 0.0;...
                -0.0000, 1.0000, 0.0].' .* 0.8;
            
         % Graph adjacency matrix for pentagon base pyramid
        adj = [ 0,  1,  0,  0,  1,  1;
                1,  0,  1,  0,  0,  1;
                0,  1,  0,  1,  0,  1;
                0,  0,  1,  0,  1,  1;
                1,  0,  0,  1,  0,  1;
                1,  1,  1,  1,  1,  0];
        
        
    case 'Cube'
        % Cube coordinates
        xyz = [1     1     1     1    -1    -1    -1    -1;
               1     1    -1    -1     1     1    -1    -1;
               1    -1     1    -1     1    -1     1    -1];

        % Graph adjacency matrix for cube
        adj = [ 0     1     1     0     1     0     0     0;
                1     0     0     1     0     1     0     0;
                1     0     0     1     0     0     1     0;
                0     1     1     0     0     0     0     1;
                1     0     0     0     0     1     1     0;
                0     1     0     0     1     0     0     1;
                0     0     1     0     1     0     0     1;
                0     0     0     1     0     1     1     0];
            
        % adj = ones(n,n) - diag(ones(1,n));    % Complete graph 
    
    case '8-agent-struct-1'
        % 8-agent-struct coordinates
        xyz = [ 0   0   0; ...
          1   0   0; ...
          1   1   0; ...
          0   1   0; ...
          0  0.5 0.5; ...
         0.5  0  0.5; ...
          1  0.5 0.5; ...
         0.5  1  0.5; ]'*1.5;

        % Graph adjacency matrix for 8-agent-struct
        adj = [ ...
                0   1   0   1   1   1   0   0    ; ...
                1   0   1   0   0   1   1   0    ; ...
                0   1   0   1   0   0   1   1    ; ...
                1   0   1   0   1   0   0   1    ; ...
                1   0   0   1   0   1   0   1    ; ...
                1   1   0   0   1   0   1   0    ; ...
                0   1   1   0   0   1   0   1    ; ...
                0   0   1   1   1   0   1   0    ]; 
    case '8-agent-struct-2'
        xyz = [ 0.    ,  0.   ,   0.;...
                1.  ,  0.     ,  0.;...
                1.  ,   1.    ,  0.;...
                0.  ,   1.    ,   0.;...
                0.  ,  0.5   , 1.0;...
                0.5    ,  0. ,  1.0;...
                1.  , 0.5,  1.0;...
                0.5    ,  1. ,  1.0].' .* 1.0;
        adj = [...
            0 ,  1 ,  0 ,  1 ,  1 ,  1 ,  0 ,  0; ...
            1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1 ,  0; ...
            0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  1; ...
            1 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1; ...
            1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  1; ...
            1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0; ...
            0 ,  1 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1; ...
            0 ,  0 ,  1 ,  1 ,  1 ,  0 ,  1 ,  0];
    case '12-agent-struct-1'
       xyz=  [ ... 
       -0.7500   ,      0      ,   0 ; ...
        0.7500   ,      0      ,   0 ; ...
             0   ,-0.7500      ,   0 ; ...
             0   , 0.7500      ,   0 ; ...
       -0.3000,    0.3000   , 0.3000 ; ...
        0.3000,    0.3000   , 0.3000 ; ...
        0.3000,   -0.3000   , 0.3000 ; ...
       -0.3000,   -0.3000   , 0.3000 ; ...
       -0.7500   , 0.7500      ,   0 ; ...
        0.7500   , 0.7500      ,   0 ; ...
        0.7500   ,-0.7500      ,   0 ; ...
       -0.7500   ,-0.7500      ,   0 ].' .* 1.0;
   
   adj = [ ...
        0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  0 ,  1 ; ...
        0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  1 ,  0 ; ...
        0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  1 ; ...
        0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  1 ,  0 ,  0 ; ...
        0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  1 ,  0 ,  0 ,  0 ; ...
        0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ,  0 ; ...
        0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ; ...
        0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  0 ,  0 ,  1 ; ...
        1 ,  0 ,  0 ,  1 ,  1 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  1 ; ...
        0 ,  1 ,  0 ,  1 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ,  0 ; ...
        0 ,  1 ,  1 ,  0 ,  0 ,  0 ,  1 ,  0 ,  0 ,  1 ,  0 ,  1 ; ...
        1 ,  0  , 1 ,  0   ,0   ,0  , 0  , 1 ,  1  , 0 ,  1  , 0 ];
    
    case '16-agent-scifi'
       
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
        xyz = [qDes ; q_in;  q_out] * 1.0;
        xyz = xyz';


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

        

    otherwise
        error('Desired formation is not defined.')
end

%%

qDes    = xyz;                  % Desired formation coordinates    
n       = size(qDes,2);         % Number of agents    

rng(0,'simdTwister');           % Reset the random number generator (for repeatability of the results)


switch formation
    case '8-agent-struct'
        qInit= [-0.5, 1.5, 0.5; ...
              -1.0, 1.5, 0.55;...
              0.5, 1.0, 0.45;...
              0.0, 1.0, 0.5;...
              0.0, 0.5, 0.54;...
              0.5, 0.0, 0.53;...
              0.5, -1.0, 0.48;...
              0.5, -1.5, 0.49]';
    otherwise
        qInit   = rand(3,n) * 5;        % Random initial positions
end

        
    



%% Inter-agent distances in the desired formation:
%
% Element (i,j) in matrix Dd describes the distance between agents i and j 
% in the formation. The diagonals are zero by definition.
Dd = zeros(n,n); 
for i = 1 : n
    for j = i+1 : n
        Dd(i,j) = norm(qDes(:,i)-qDes(:,j), 2);
    end
end
Dd = Dd + Dd';





































































































