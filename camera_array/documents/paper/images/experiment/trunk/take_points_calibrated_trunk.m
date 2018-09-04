clear all 
close all

frame_calibrated={ 'Exp_12_img_calibrated_0000000102.jpg', 'Exp_13_img_calibrated_0000000010.jpg', 'Exp_13_img_calibrated_0000000247.jpg', 'Exp_13_img_calibrated_0000000490.jpg'};

frame_ndvi={ 'Exp_12_ndvi_1D_0000000102.jpg', 'Exp_13_ndvi_1D_0000000010.jpg', 'Exp_13_ndvi_1D_0000000247.jpg', 'Exp_13_ndvi_1D_0000000490.jpg'};

previus_size=0;
x_total=[];

for frame=1:size(frame_calibrated,2)

    calibrated=frame_calibrated{frame};
    ndvi=frame_ndvi{frame};

    %%%%%%%%%%% take points %plot%%%%%%%%%%%%%%%%

    imshow(calibrated); 

    [x,y,value_calibrated] = impixel;

    im_ndvi = imread(ndvi);

    ndvi_value=zeros(size(x,1),1);

    for i=1: size(x,1)
        ndvi_value(i,1)= im_ndvi(y(i),x(i));
    end

    %%%%%%%%%% save data %%%%%%%%%%%%%%%%%%%

    for i=1:size(x,1)
        x_total(previus_size+i,1) = x(i);
        y_total(previus_size+i,1) = y(i);
        ndvi_total(previus_size+i,1) = ((ndvi_value(i)/128)-1)*2 -1;
    end

    previus_size=size(x_total,1);


end

data = [x_total y_total ndvi_total];

dlmwrite('data_trunk.txt',data);