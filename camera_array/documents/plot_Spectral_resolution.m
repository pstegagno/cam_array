clear all 
close all 

data=load('filter_trasmission_plus_camera.txt');
data1=load('filter_trasmission.txt');

figure(1)
hold on
grid on
plot(data1(data1(:,4)>0,1), data1(data1(:,4)>0,4), 'r-+','MarkerSize',6);
plot(data1(data1(:,6)>1,1), data1(data1(:,6)>1,6),'g-x','MarkerSize',6);
plot(data1(:,1), data1(:,7),'k--' ); % camera
plot(data(data(:,4)>1,1), data(data(:,4)>1,4),'m-s', 'Color', [0.5 0 0], 'MarkerSize',4);
plot(data(data(:,6)>1,1), data(data(:,6)>1,6), 'g-o','Color', [0 0.5 0],'MarkerSize',4 );


xlabel('wavelength [nm]')
ylabel('efficiency [%]')
%legend('S_{red}', 'S_{NIR}', 'S_c', 'S_{cRed}=S_{red}+S_c','S_{cNIR}=S_{NIR+S_c}' )
legend('TE_{red}', 'TE_{NIR}', 'QE', 'TotE_{NIR}' )