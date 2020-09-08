function F = bounding_box (img, myDepthMat,m,result_Boundary_Folder)
se = strel("disk",4,0);
ne =  getnhood(se);
bwImg = img;
for i=1:4
  bwImg = imerode(bwImg, ne);
end
for i=1:4
    bwImg = imdilate(bwImg, ne);
end

img = bwImg;

% Region labeling
L = bwlabel(bwImg, 8);

s = regionprops(L);
figure
imshow(L);
hold on
stats = regionprops(L,'Centroid');
centers= cat(1,stats.Centroid);
numBlob = numel(s);
BW_filled = imfill(L,'holes');
boundaries = bwboundaries(BW_filled);
avg = zeros(1,numBlob);
for k=1:numBlob
    img2 = myDepthMat.*(L == k);
    b = boundaries{k};
    plot(b(:,2),b(:,1),'g','LineWidth',3);
    avg(k) = mean(img2(img2>0));
    c = int2str(uint16(avg(k))); 
    final = text(centers(k,1),centers(k,2), c, 'Color',"cyan");
end
%saveas(gca,fullfile('test.jpg');
filename=strcat('resultbackground_',num2str(m));
filesave = fullfile(result_Boundary_Folder, filename);
saveas(gca, filesave, 'jpeg');
close all
endfunction
