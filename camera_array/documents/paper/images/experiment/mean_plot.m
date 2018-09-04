clear all 
close all

snow = load('snow/data_snow.txt');
concrete = load('concrete/data_concrete.txt');
leaves = load('leaves/data_leaves.txt');
trunk = load('trunk/data_trunk.txt');
tar = load('tar/data_tar_secondCampaign.txt');
water = load('water/data_water.txt');


data = [ water(1:95,3) tar(1:95,3) snow(1:95,3) concrete(1:95,3) trunk(1:95,3) leaves(1:95,3) ];

boxplot(data, {'Water', 'Tar', 'Snow', 'Concrete', 'Plant stem', 'Leaves'})
ylabel('NDVI')
