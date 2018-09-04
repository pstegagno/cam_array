clear all 
close all

%%%%%%%%%%%take points%%%%%%%%%%%%%%%%%
imshow 'camera2_0000002600.jpg'


[x,y,value_nir] = impixel;

im_ndvi = imread('ndvi_1D_0000000187.jpg');

ndvi_value=zeros(size(x,1),1);

for i=1: size(x,1)
    ndvi_value(i,1)= im_ndvi(ceil(y(i)/2), ceil(x(i)/2));
end



%%%%%%%%%%%%make plots%%%%%%%%%%%%%%
subplot(2,1,1)
%im_ndvi = imread('ndvi_1D_0000000187.jpg');
image(im_ndvi/1.5)
hold on
for i=1: size(x,1)
    plot(x(i)/2,y(i)/2,'ro','MarkerSize',10)
    text(x(i)/2,y(i)/2,num2str(i))
end
axis off 

subplot(2,1,2)
im_nir = imread('camera2_0000002600.jpg');
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

data=[x y value_nir(:,1) ndvi_value];

dlmwrite('data.txt',data);









 