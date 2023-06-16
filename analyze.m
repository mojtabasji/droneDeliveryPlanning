close all
clear all
clc

LinesCount = 9;
VCount = 60;
depot_count = 4;
approach = 'deep'; 



[dataFile, dataFileString] = xlsread('operations_'+ string(VCount)+'-'+string(depot_count)+'.xls' );
moredata = xlsread('moreData'+ string(VCount)+'-'+string(depot_count)+'.xls' );
save('vars.mat', 'dataFile', 'dataFileString', 'moredata');
% load 'vars.mat';
er = moredata(:,3);
indexs = find(~isnan(dataFile(:,6)));
part1length = length(indexs);

part1 = dataFile(indexs, 1:8);
part1String = dataFileString(indexs, 1:8);
part2 = dataFile(2:end, 11:6 * LinesCount + 10);

sumOfReachTime = part1(:,1) + part1(:,2) + part1(:,3) + part1(:,4);
subplot(3,1,[1:2 1])
plot( 1:part1length/2 , sumOfReachTime(1:part1length/2), 1:part1length/2, part1(1:part1length/2,2));
xlabel('Deliver Number', 'FontSize',16)
ylabel('Time (Seconds)', 'FontSize',16)
legend('Total Serve Time', 'Waiting Time', 'FontSize', 16);
set(gca,'FontSize',16)
annotation('textbox', [0.30, 0.8, 0.1, 0.1], 'String', "Served Pakage Count:  " + fix(part1length / 2) + newline +"UAV : " + VCount, 'FontSize',16)

disp("avg waiting:");
disp(mean(part1(find(~isnan(part1(:,2))),2))/60);
disp("avg deliver:");
disp(mean(sumOfReachTime(find(~isnan(sumOfReachTime(1:part1length)))))/60);

%  the same as above
% disp("avg waiting:");
% waitings = part1(find(~isnan(part1(:,2))),2);
% disp(sum(waitings) / length(waitings));

subplot(3,1,3)
plot(1:length(er), er);
% title('exploration rate');

firstDataLine =3;
firstDataParam = 3; %{1: fly2, 2:fly2R, 3:wait, 4:wait_R, 5:onLine, 6:online_R}

towiceDataLine = 3;
towiceDataParam = 4; %{1: fly2, 2:fly2R, 3:wait, 4:wait_R, 5:onLine, 6:online_R}
figure
indexs = find(~isnan(part2(:,1)));
L0onLine = part2(indexs,(firstDataLine-1)*6 + firstDataParam);
L1onLine = part2(indexs,(towiceDataLine -1)*6 + towiceDataParam);
plot( 1:length(indexs) , L0onLine, 1:length(indexs), L1onLine);
xlabel('Seconds')
ylabel('UAV Count')
legend("Line "+firstDataLine, "Line "+towiceDataLine);
% title('on Line');
% map = imread('runConf.png');
% subplot(1,3,[1, 1:2]);
% % imshow(map);
% f = fileread('config.txt');
% subplot(1,3,3, 'Visible', 'off');
% text( .05,0.7, f, 'VerticalAlignment', 'bottom');



lines = ["line1", "line2", "line3", "line4", "line5", "line6", "line7", "line8", "line9", "line10"];
lines_count = zeros(1,10);

for i = 1:part1length
   for j = 1:10
      if strcmp(part1String(i,7),  lines(j))
         lines_count(j) = lines_count(j) + 1; 
      end
   end
end
    
figure
bar(lines_count);
save('lines_16.mat', 'lines_count')
xlabel("Line number");
ylabel("use count");
grid on;




