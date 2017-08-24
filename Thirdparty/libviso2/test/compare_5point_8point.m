function compare_5point_8point(entryId)
% compare monocular [R,t] obtained by five point and eight point algorithm
% entryId is 1-based index of an entry in the 3X4 P=[R|t] matrix, P is transform
% from previous to current frame.
data1= load('F:\mono_opencv5point.txt');% generated using libviso2
data2=load('F:\mono_viso8point.txt');
addpath('C:\JianzhuHuai\GPS_IMU\programs\matlab_ws\utilities');
data3= load('F:\kitti\data_odometry_poses\dataset\poses\01.txt');
truth= data3;
for jack=2:size(data3,1)
    truth(jack, :)= getPprev2curr(data3(jack-1,:), data3(jack, :));
end
nextFig=0;
nextFig=nextFig+1;
figure(nextFig);
plot(data1(:,1), data1(:, entryId+1),'r', data2(:,1), data2(:,entryId+1), 'g', (0:size(truth,1)-1)', truth(:, entryId),'b');
tit=sprintf('entry %d', entryId);
title(tit);
legend('opencv','viso', 'truth');
    function vTprev2curr= getPprev2curr(vTprev2w, vTcurr2w)
        %Tprev2w and Tcurr2w 12x1, row major
        Tprev2w= reshape(vTprev2w, 4,3)';
        Tprev2w= [Tprev2w; 0,0,0,1];
        Tcurr2w= reshape(vTcurr2w, 4,3)';
        Tcurr2w= [Tcurr2w; 0,0,0,1];
        Tprev2curr=inverseP(Tcurr2w)*Tprev2w;
        vTprev2curr= reshape(Tprev2curr(1:3,:)', 1, 12);
    end
end
