clear all
close all 

frame_calibrated={ 'Exp_32_img_calibrated_0000000067.png', 'Exp_32_img_calibrated_0000000157.png', 'Exp_33_img_calibrated_0000000109.png'};

frame_ndvi={ 'Exp_32_ndvi_1D_0000000067.png', 'Exp_32_ndvi_1D_0000000157.png', 'Exp_33_ndvi_1D_0000000109.png'};

previus_size=0;
x_total=[];

for frame=1:size(frame_calibrated,2)

    calibrated=frame_calibrated{frame};
    ndvi=frame_ndvi{frame};

    %%%%%%%%%%% take points plot %%%%%%%%%%%%%%%%

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

dlmwrite('data_tar_secondCampaign.txt',data);



