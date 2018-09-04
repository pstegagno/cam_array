clear all 
close all

%%%%%%%%%%%take points%%%%%%%%%%%%%%%%%
imshow 'img_calibrated_0000000017.jpg'


[x,y,value_calibrated] = impixel;

im_ndvi = imread('ndvi_1D_0000000017.jpg');

ndvi_value=zeros(size(x,1),1);

for i=1: size(x,1)
    ndvi_value(i,1)= im_ndvi(y(i),x(i));
end



%%%%%%%%%%%%make plots%%%%%%%%%%%%%%
subplot(2,1,1)
%im_ndvi = imread('ndvi_1D_0000000187.jpg');
image(im_ndvi/1.5)
hold on
for i=1: size(x,1)
    plot(x(i),y(i),'ro','MarkerSize',10)
    text(x(i),y(i),num2str(i))
end
axis off 

subplot(2,1,2)
value_calibrated = imread('img_calibrated_0000000017.jpg');
%figure(2)
hold off
image(im_nir)
hold on

for i=1: size(x,1)
    plot(x(i),y(i),'ro','MarkerSize',10)
    text(x(i),y(i),num2str(i))
end
axis off 

%%%%%%%%%%%%save data%%%%%%%%%%%%

data=[x y value_calibrated(:,1) ndvi_value];

%dlmwrite('data.txt',data);