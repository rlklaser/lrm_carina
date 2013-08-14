%generates motion primitives and saves them into file

resolution = 0.1;          	% resolucao da primitiva
numberofangles = 16; 		% número de fracoes de angulos para um giro de 360 graus
numberofprimsperangle = 9; 	% numero de primitivas de movimento

%numofsamples = 30;          % numero de passos para cada primitiva (poses calculadas)

custo0 = 1;
custo1 = 2.5;
custo2 = 4;
custo3 = 7;

stlim = 0.66;  % estercamento maximo 38 graus
vmax = 2;      % velocidade maxima
L = 1.64;      % distancia entre eixos
dt = 0.1;

%maxsteer = stlim;

% angulos devem ser dentro da fracao do giro
% [ end_x | end_y | end_pose | cost | speed | steer | samples]

%basemprimendpts1_c(8 ,:)  =  [ -4   0   4   custo2   0.3   -maxsteer/1  16];
%basemprimendpts1_c(6 ,:)  =  [ -6   0   3   custo2   0.4   -maxsteer/2  20];
%basemprimendpts1_c(4 ,:)  =  [ -8   0   2   custo2   0.5   -maxsteer/4  26];
%basemprimendpts1_c(2 ,:)  =  [-10   0   1   custo2   0.6   -maxsteer/8  30];
%basemprimendpts1_c(1 ,:)  =  [ -5   0   0   custo1   0.8    0           10];
%basemprimendpts1_c(3 ,:)  =  [-10   0  -1   custo2   0.6    maxsteer/8  30];
%basemprimendpts1_c(5, :)  =  [ -8   0  -2   custo2   0.5    maxsteer/4  26];
%basemprimendpts1_c(7 ,:)  =  [ -6   0  -3   custo2   0.4    maxsteer/2  20];
%basemprimendpts1_c(9 ,:)  =  [ -4   0  -4   custo2   0.3    maxsteer/1  16];


%%% RE %%%

%basemprimendpts0_c( 8,:)  =  [ -16  -2   1   custo1  -0.4  -pi/12  40];
%basemprimendpts0_c( 9,:)  =  [ -15  -4   1   custo1  -0.4  -pi/8.2 40];
%basemprimendpts0_c(10,:)  =  [  -6   0   0   custo1  -0.4   0      40];
%basemprimendpts0_c(11,:)  =  [ -16   2  -1   custo1  -0.4   pi/12  40];
%basemprimendpts0_c(12,:)  =  [ -15   4  -1   custo1  -0.4   pi/8.2 40];
%basemprimendpts0_c(13,:)  =  [  -8  -1   1   custo3  -0.2  -pi/8.2 40];
%basemprimendpts0_c(14,:)  =  [  -8   1  -1   custo3  -0.2   pi/8.2 40];

%pi/8.2 = 0.3831 = 22 deg
%pi/12  = 0.2618 = 15 deg

basemprimendpts0_c(1,:) = [ 15  -4  -1  custo1  0.4  -pi/8.2 40];
basemprimendpts0_c(2,:) = [ 16  -2  -1  custo1  0.4  -pi/12  40];
basemprimendpts0_c(3,:) = [ 10   0   0  custo0  0.4   0      40];
basemprimendpts0_c(4,:) = [ 16   2   1  custo1  0.4   pi/12  40];
basemprimendpts0_c(5,:) = [ 15   4   1  custo1  0.4   pi/8.2 40];

basemprimendpts0_c(6,:) = [  8  -2  -1  custo2  0.3  -pi/8.2 40];
basemprimendpts0_c(7,:) = [  8   2   1  custo2  0.3   pi/8.2 40];

basemprimendpts0_c(8,:) = [  7  -3  -1  custo3  0.2  -stlim 50];
basemprimendpts0_c(9,:) = [  7   3   1  custo3  0.2   stlim 50];

    
fout = fopen('../carina.mprim', 'w');
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
for angleind = 1:numberofangles
    figure(1);
    hold off;
    text(0, 0, int2str(angleind));
    for primind = 1:numberofprimsperangle

        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);

        %current angle
        angle = (angleind-1)*2*pi/numberofangles;
        
        basemprimendpts_c = basemprimendpts0_c(primind, :);    
         
        baseendpose_c =            basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);          
        speed =                    basemprimendpts_c(5);
        steer =                    basemprimendpts_c(6);
        numofsamples =             basemprimendpts_c(7);
 
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        %generate intermediate poses       
        intermcells_m = zeros(numofsamples, 3);
        startpt = [0 0 angle];
        endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles steer];
              
        intermcells_m = zeros(numofsamples, 3);
        
        for i=1:numofsamples
            next= car(speed, steer, startpt, stlim, vmax, L, dt);
            intermcells_m(i, :)=next;
            startpt=next;
        end     
               
        errorxy = [endpt(1) - intermcells_m(numofsamples, 1) ... 
                   endpt(2) - intermcells_m(numofsamples, 2) ...
                   endpt(3) - intermcells_m(numofsamples, 3)];
                   
        fprintf(1, 'err_x = %5f err_y = %5f err_theta = %5f\n', errorxy(1), errorxy(2) , errorxy(3));
        
        interpfactor = [0:1/(numofsamples-1):1];
        intermcells_m(:,1) = intermcells_m(:, 1) + errorxy(1)*interpfactor';
        intermcells_m(:,2) = intermcells_m(:, 2) + errorxy(2)*interpfactor';
        intermcells_m(:,3) = intermcells_m(:, 3) + errorxy(3)*interpfactor';
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind, 1), ...
                    intermcells_m(interind, 2), intermcells_m(interind, 3));
        end;
        
        hold on;
        plot(intermcells_m(:,1), intermcells_m(:,2));
        axis([-2 2 -2 2]);
        text(intermcells_m(numofsamples, 1), intermcells_m(numofsamples, 2), int2str(endpose_c(3)));
        hold on;
        
    end; 

    grid;
    %if angleind == 1
    pause;
    %end;
    clc;

end;

fclose(fout);
close('all');

