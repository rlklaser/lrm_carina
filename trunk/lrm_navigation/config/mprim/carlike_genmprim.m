%generates motion primitives and saves them into file

resolution = 0.1;          % resolucao
%numberofangles = 24;        % número de angulos
numberofangles = 16;
numberofprimsperangle = 14; % numero de primitivas por angulo

numofsamples = 40;
 
custo1 = 1;
custo2 = 2;
custo3 = 5;
custo4 = 2000;

%$$$$$$$$$$$$$$$$$$$$$$$$$$    0 degreees   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
% basemprimendpts0_c = zeros(numberofprimsperangle, 6);
% basemprimendpts0_c(1 ,:)  =  [ 24   -2   -1     custo1   0.3    -pi/12];
% basemprimendpts0_c(2 ,:)  =  [ 23   -5   -1     custo1   0.3    -pi/6 ];
% basemprimendpts0_c(3 ,:)  =  [ 10    0    0      custo1   0.3      0   ];  
% basemprimendpts0_c(4 ,:)  =  [ 24    2    1     custo1   0.3     pi/12];
% basemprimendpts0_c(5 ,:)  =  [ 23    5    1     custo1   0.3     pi/6 ];
% basemprimendpts0_c(6,:)  =  [ -8   -1    -1     custo3   0.1     -pi/6 ];
% basemprimendpts0_c(7,:)  =  [ -8    1     1     custo3   0.1      pi/6 ];

% basemprimendpts0_c(8 ,:)  =  [ 32   -4   -1     custo2   0.4    -pi/12];
% basemprimendpts0_c(9 ,:)  =  [ 30   -8   -2     custo2   0.4    -pi/6 ];
% basemprimendpts0_c(10 ,:)  =  [ 12    0    0     custo2   0.4      0   ];
% basemprimendpts0_c(11 ,:)  =  [ 32    4    1     custo2   0.4     pi/12];
% basemprimendpts0_c(12,:)  =  [ 30    8    2     custo2   0.4     pi/6 ];
% basemprimendpts0_c(13,:)  =  [ 10   -1    -1     custo3    0.1     -pi/6 ];
% basemprimendpts0_c(14,:)  =  [ 10    1    1     custo3    0.1      pi/6 ];
    
% basemprimendpts0_c(6,:)  =  [ 50   -10   -1     custo3   0.5    -pi/12];
% basemprimendpts0_c(7,:)  =  [ 44   -20  -2     custo3   0.5    -pi/6 ];
% basemprimendpts0_c(8,:)  =  [ 20    0    0     custo3   0.5      0   ];
% basemprimendpts0_c(9,:)  =  [ 50    10    1     custo3   0.5     pi/12];
% basemprimendpts0_c(10,:)  =  [ 44    20   2     custo3   0.5     pi/6 ];
     
% basemprimendpts0_c = zeros(numberofprimsperangle, 6);
% basemprimendpts0_c(1 ,:)  =  [ -24   -2   -1     custo1   0.3    -pi/12];
% basemprimendpts0_c(2 ,:)  =  [ -23   -5   -1     custo1   0.3    -pi/6 ];
% basemprimendpts0_c(3 ,:)  =  [ -10    0    0     custo1   0.3      0   ];  
% basemprimendpts0_c(4 ,:)  =  [ -24    2    1     custo1   0.3     pi/12];
% basemprimendpts0_c(5 ,:)  =  [ -23    5    1     custo1   0.3     pi/6 ];
% basemprimendpts0_c(6,:)  =  [ -8   -1    -1     custo3   0.1     -pi/6 ];
% basemprimendpts0_c(7,:)  =  [ -8    1     1     custo3   0.1      pi/6 ];
    
% basemprimendpts0_c(1 ,:)  =  [ -32   -4   1     custo2   0.4    -pi/12];
% basemprimendpts0_c(2 ,:)  =  [ -30   -8   2     custo2   0.4    -pi/6 ];
% basemprimendpts0_c(3 ,:)  =  [ -12    0    0     custo2   0.4      0   ];
% basemprimendpts0_c(4 ,:)  =  [ -32    4    -1     custo2   0.4     pi/12];
% basemprimendpts0_c(5, :)  =  [ -30    8    -2     custo2   0.4     pi/6 ];
% basemprimendpts0_c(6,:)  =  [ -10   -1    1     custo3    0.1     -pi/6 ];
% basemprimendpts0_c(7,:)  =  [ -10    1    -1     custo3    0.1      pi/6 ];

% basemprimendpts0_c(6,:)  =  [ -50   -10   -1     custo3   0.5    -pi/12];
% basemprimendpts0_c(7,:)  =  [ -44   -20  -2     custo3   0.5    -pi/6 ];
% basemprimendpts0_c(8,:)  =  [ -20    0    0     custo3   0.5      0   ];
% basemprimendpts0_c(9,:)  =  [ -50    10    1     custo3   0.5     pi/12];
% basemprimendpts0_c(10,:)  = [ -44    20   2     custo3   0.5     pi/6 ];

% basemprimendpts0_c(1,:)  =  [ -12   -1   -3     custo3   0.2    -pi/12];
% basemprimendpts0_c(2,:)  =  [ -8   -1   -1      custo3   0.2    -pi/12 ];
% basemprimendpts0_c(3,:)  =  [ -5    0    0     custo3   0.2      0   ];
% basemprimendpts0_c(4,:)  =  [ -12    1    3     custo3   0.2     pi/12];
% basemprimendpts0_c(5,:)  =  [ -8   1    1     custo3   0.2     pi/12 ];

basemprimendpts0_c(1 ,:)  =  [-16  -2   1   custo1   -0.4   -pi/12 ];
basemprimendpts0_c(2 ,:)  =  [-15  -4   1   custo1   -0.4   -pi/8.2];
basemprimendpts0_c(3 ,:)  =  [ -6   0   0   custo1   -0.4      0   ];
basemprimendpts0_c(4 ,:)  =  [-16   2  -1   custo1   -0.4    pi/12 ];
basemprimendpts0_c(5 ,:)  =  [-15   4  -1   custo1   -0.4    pi/8.2];
basemprimendpts0_c(6 ,:)  =  [ -8  -1   1   custo3   -0.2   -pi/8.2];
basemprimendpts0_c(7 ,:)  =  [ -8   1  -1   custo3   -0.2    pi/8.2];

basemprimendpts0_c(8 ,:)  =  [16   -2  -1   custo1   0.4    -pi/12 ];
basemprimendpts0_c(9 ,:)  =  [15   -4  -1   custo1   0.4    -pi/8.2];
basemprimendpts0_c(10,:)  =  [ 6    0   0   custo1   0.4      0    ];
basemprimendpts0_c(11,:)  =  [16    2   1   custo1   0.4     pi/12 ];
basemprimendpts0_c(12,:)  =  [15    4   1   custo1   0.4     pi/8.2];
basemprimendpts0_c(13,:)  =  [ 8   -1  -1   custo3   0.2    -pi/8.2];
basemprimendpts0_c(14,:)  =  [ 8    1   1   custo3   0.2     pi/8.2];

fout = fopen('out/carina10cmFrenteRe16.mprim', 'w');
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
for angleind = 1:numberofangles  % 16 angulos
    figure(1);
    hold off;
    text(0, 0, int2str(angleind));
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);

        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        basemprimendpts_c = basemprimendpts0_c(primind,:);    
        angle = currentangle;    
        baseendpose_c = basemprimendpts_c(1:3);
        speed = basemprimendpts_c(5);
        steer = basemprimendpts_c(6);
        additionalactioncostmult = basemprimendpts_c(4);        
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];

        %generate intermediate poses
        intermcells_m = zeros(numofsamples,3);
        startpt = [ 0  0  currentangle];
        
        endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
              rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles steer];
              
        intermcells_m = zeros(numofsamples,3);
        
        for i=1:numofsamples
            next= car(speed,steer,startpt);
            intermcells_m(i,:)=next;
            startpt=next;
        end     
               
        errorxy = [endpt(1) - intermcells_m(numofsamples,1) ... 
        endpt(2) - intermcells_m(numofsamples,2) ...
        endpt(3) - intermcells_m(numofsamples,3)];
        
        fprintf(1, ' errx=%f erry=%f\n errteta=%f\n', errorxy(1), errorxy(2) , errorxy(3));
        interpfactor = [0:1/(numofsamples-1):1];
        intermcells_m(:,1) = intermcells_m(:,1) + errorxy(1)*interpfactor';
        intermcells_m(:,2) = intermcells_m(:,2) + errorxy(2)*interpfactor';
        intermcells_m(:,3) = intermcells_m(:,3) + errorxy(3)*interpfactor';
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), ...
            intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        hold on;
        plot(intermcells_m(:,1), intermcells_m(:,2));
        axis([-2 2 -0.5 0.5]);
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;
    grid;
    pause;
    clc;
end;
        
fclose('all');
