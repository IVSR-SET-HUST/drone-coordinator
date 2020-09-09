clear, clc, close all
%graphics_toolkit("gnuplot");
%pkg load image;
myFolder = '/home/luongmanh/lab/python/getdata/data1/depth_mat/';
filePattern = fullfile(myFolder, '*.mat');
jpegFiles = dir(filePattern);
resultFolder = '/home/luongmanh/lab/octave/result1/';
result_Boundary_Folder = '/home/luongmanh/lab/octave/result1_bound/';
d = zeros(length(jpegFiles),480,640);
f = d;
b = d;
for k = 1: length(jpegFiles)
    baseFileName = jpegFiles(k).name;
    fullFileName = fullfile(myFolder, baseFileName);
    load(fullFileName);
end
for k = 1: length(jpegFiles)
    a{k} = eval(strcat('depth',num2str(k)));
    d(k,:,:)=reshape(a{k},[1,480,640]);
end
[~, rows, columns] = size(d);

alpha_b = 0.01;
b(1,:,:) = d(1,:,:);
thresh = 500;
for k = 2: length(jpegFiles)
    b(k,:,:) = (1 - alpha_b) * b(k-1,:,:) + alpha_b * d(k-1, :,:);
    f(k,:,:) = (b(k,:,:) - d(k,:,:))>thresh;
    img1 = reshape(f(k,:,:),[rows, columns]);
    img = uint8(255 * mat2gray(img1));
    filename=strcat('background_',num2str(k),'.jpg');
    filesave = fullfile(resultFolder, filename);
    imwrite(img1, filesave); 
    bounding_box(img1,a{k},k,result_Boundary_Folder);
end

%imshow(uint8(255 * mat2gray(reshape(f(1,:,:),[rows, columns]))))