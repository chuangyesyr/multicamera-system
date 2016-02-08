clc; clear;
NO = 8;
c = [1 0 1 0 0 1 1 1];

lookuptable1 = [111;222;333;444;555;666;777;888];
lookupdistance1 = [1;2;3;4;5;6;7;8];




for ii = 1:NO
    if c(1,ii) == 1
        for jj = ii+1:NO
            if c(1,jj) == 1
                min1 = min(savedAngles(ii,:));
                max1 = max(savedAngles(ii,:));
                min2 = min(savedAngles(jj,:));
                max2 = max(savedAngles(jj,:));
            end
        end
    end
end
end



%   c = a.*b;
%     index = c == 1;
%     Table(i, index) = 1; % FOV table
%
%     for ii = 1:NO
%
%     min1 = min(savedAngles(1,:));
%     max1 = max(savedAngles(1,:));
%     min2 = min(savedAngles(2,:));
%     max2 = max(savedAngles(2,:));
%
% %     c1 = c;
%
%     if max2 > min1 && min2 < max1
%         if savedDistance(1,1) < savedDistance(2,1)
%             c1(1,2) = 1;
%         else c1(1,1) = 2;
%         end
%     end