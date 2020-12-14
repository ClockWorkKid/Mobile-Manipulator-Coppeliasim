%{

This function is always called at a junction where we have to check if our 
desired room is on the left or right wall from that junction. The function takes
as input a given frame(L/R) and the desired destination room. If it finds
the color of the desired destination room on the wall, then it returns 1,
otherwise it returns 0.

Author: Shafin Bin Hamid 

%}

function result = check_wall(self, frame, dest_point)

frame_hsv = rgb2hsv(frame);

h = frame_hsv(:,:,1);
s = frame_hsv(:,:,2);
v = frame_hsv(:,:,3);
%pause()

frame_final = zeros(size(frame_hsv,1), size(frame_hsv,2));

switch(dest_point)
    case 5 %RED
        for i=1:size(frame_hsv,1)
            for j=1:size(frame_hsv,2)
                if(h(i,j)==0) && s(i,j)>=0.75 && s(i,j)<=0.76 && v(i,j)>=0.49 && v(i,j)<= 0.51
                    frame_final(i,j) = 1;
                end
            end
        end
    case 6 %OLIVE
        for i=1:size(frame_hsv,1)
            for j=1:size(frame_hsv,2)
                if h(i,j)>=0.16 && h(i,j)<=0.17 && s(i,j)>=0.57 && s(i,j)<=0.58 && v(i,j)>=0.45 && v(i,j)<= 0.46
                    frame_final(i,j) = 1;
                end
            end
        end
    case 3 %GREEN
        for i=1:size(frame_hsv,1)
            for j=1:size(frame_hsv,2)
                if h(i,j)>=0.32 && h(i,j)<=0.34 && s(i,j)>=0.57 && s(i,j)<=0.58 && v(i,j)>=0.44 && v(i,j)<= 0.45
                    frame_final(i,j) = 1;
                end
            end
        end
    case 4 %CYAN
        for i=1:size(frame_hsv,1)
            for j=1:size(frame_hsv,2)
                if h(i,j)>=0.49 && h(i,j)<=0.51 && s(i,j)>=0.57 && s(i,j)<=0.58 && v(i,j)>=0.45 && v(i,j)<= 0.46
                    frame_final(i,j) = 1;
                end
            end
        end
    case 1 %BLUE
        for i=1:size(frame_hsv,1)
            for j=1:size(frame_hsv,2)
                if h(i,j)>=0.66 && h(i,j)<=0.67 && s(i,j)>=0.57 && s(i,j)<=0.58 && v(i,j)>=0.44 && v(i,j)<= 0.45
                    frame_final(i,j) = 1;
                end
            end
        end
    case 2 %PURPLE
        for i=1:size(frame_hsv,1)
            for j=1:size(frame_hsv,2)
                if h(i,j)>=0.82 && h(i,j)<=0.84 && s(i,j)>=0.57 && s(i,j)<=0.58 && v(i,j)>=0.45 && v(i,j)<= 0.46
                    frame_final(i,j) = 1;
                end
            end
        end
        
end

% subplot(131), imshow(frame)
% subplot(132), imshow(frame_hsv)
% subplot(133), imshow(frame_final)
%pause()

stats = regionprops(frame_final);
%pause()
if isempty(stats)
    result = 0;
elseif stats.Area < 100
    result = 0;
else
    result = 1;
end

end