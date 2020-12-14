%%
frame_front = rgb2gray(self.frame_front);
frame_left = rgb2gray(self.frame_left);
frame_right = rgb2gray(self.frame_right);

%%
frame_front_binarized = imbinarize(frame_front);
frame_left_binarized = imbinarize(frame_left);
frame_right_binarized = imbinarize(frame_right);

%%
%disp(frame_front);
%debugging, before and after grayscaling
%     figure
%     subplot(231), imshow(self.frame_left);
%     subplot(232), imshow(self.frame_front);
%     subplot(233), imshow(self.frame_right);
%     subplot(2,3,4), imshow(frame_left);
%     subplot(2,3,5), imshow(frame_front);
%     subplot(2,3,6), imshow(frame_right);
%     frame_front_binarized = imbinarize(frame_front);
%figure(1)
%subplot(1,2,1), imshow(frame_front_binarized);
%subplot(1,2,2), imshow(frame_front_binarized(1, :));

%%
sValues = zeros(1,5);
for i=1:5
    sValues(i) = mean(sensor_values((i-1)*13+1 : i*13));
    if(sValues(i)<50)
        sValues(i) = 1;
    else
        sValues(i) = 0;
    end
end















