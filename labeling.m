function   [occRate, Table, TableOcc] = labeling(Cameras_Position, Objects_Position, ObjectSize, obstacle_parameters, ObstacleSize_S)

[~, NC] = size(Cameras_Position);
[~, NO] = size(Objects_Position);
[~, NS] = size(obstacle_parameters);

a = zeros(1, NO);
b = zeros(1, NO);

if NS ~= 0
    a_s = zeros(1, NS);
    b_s = zeros(1, NS);
end

Table = zeros(NC, NO);           % Row denotes Cam and Column denotes Obj
Table_s = zeros(NC, NS);
TableOcc = zeros(NC,NO);
occRate = zeros(NC, NO);

% c1 = zeros(1,2);
savedAngles = zeros(NC, 4);
correctedAngles = zeros(NC, 4);  % angles larger than pi/3 should be seen as pi/3, angles larger than pi, should be set to zero

savedDistance = zeros(NC,1);
%% Determine if the Object j is in the FOV of Camera i;
for i = 1:NC
    a1 = Objects_Position(1, :) - ObjectSize/2;
    a2 = Objects_Position(1, :) + ObjectSize/2;
    b1 = Objects_Position(2, :) - ObjectSize/2;
    b2 = Objects_Position(2, :) + ObjectSize/2;
    
    Points = [a1 a1 a2 a2; b1 b2 b1 b2];
    Difference = Points - repmat(Cameras_Position(1:2, i), 1, 4*NO);       % difference is the vector pointing from camera to object vertices
    
    Difference1 = Difference(1, :);
    
    Difference1(Difference1 > 0) = 0;
    Difference1(Difference1 < 0) = 1;
    
    D = Difference1;
    
    Angles = mod(atan(Difference(2,:)./Difference(1,:)) + pi*D, 2*pi);
    
    if NS ~= 0
        a_s1 = obstacle_parameters(1, :) - ObstacleSize_S/2;
        a_s2 = obstacle_parameters(1, :) + ObstacleSize_S/2;
        b_s1 = obstacle_parameters(2, :) - ObstacleSize_S/2;
        b_s2 = obstacle_parameters(2, :) + ObstacleSize_S/2;
        
        Points_s = [a_s1 a_s1 a_s2 a_s2; b_s1 b_s2 b_s1 b_s2];
        Difference_s = Points_s - repmat(Cameras_Position(1:2, i), 1, 4*NS);       % difference is the vector pointing from camera to object vertices
    
        Difference_s1 = Difference_s(1, :);
    
        Difference_s1(Difference_s1 > 0) = 0;
        Difference_s1(Difference_s1 < 0) = 1;
    
        D_s = Difference_s1;
    
        Angles_s = mod(atan(Difference_s(2,:)./Difference_s(1,:)) + pi*D_s, 2*pi);
    end
    

    

    
    for j = 1:NO
        Angle = [Angles(j), Angles(j + NO), Angles(j + 2*NO), Angles(j + 3*NO)];
        %         if (Camera_Position(3, i) < pi/3)
        Angle1 = mod((Angle - mod((Cameras_Position(3, i) - Cameras_Position(4, i)/2), 2*pi)), 2*pi);
        a(j) = ~isempty(find((Angle1 >= 0) & (Angle1 <= Cameras_Position(4, i)), 1));
        %         a(j) = ~isempty(find((Angle > (Cameras_Position(3, i) - Cameras_Position(4, i)/2)) & ...,
        %                          (Angle < (Cameras_Position(3, i) + Cameras_Position(4, i)/2)), 1));
        b(j) = boolean(norm(Objects_Position(:,j) - Cameras_Position(1:2, i)) < Cameras_Position(5, i));
        
        if (abs(Objects_Position(1, j)-Cameras_Position(1, i)) < ObjectSize/2 && abs(Objects_Position(2, j)-Cameras_Position(2, i)) < ObjectSize/2 )
            a(j) = 1;
        end
        
        Distances(j, i) = norm(Objects_Position(:,j) - Cameras_Position(1:2, i));
        
        %% occlusion table
        savedAngles(j,:) = Angle1;
        correctedAngles(j,:) = Angle1;
        savedDistance(j,:) = norm(Objects_Position(:,j) - Cameras_Position(1:2, i));
        
        for cj = 1:4                                      % correct angles to be in FOV angles
            if correctedAngles(j,cj)>pi
                correctedAngles(j,cj) = 0;
            elseif correctedAngles(j,cj) > pi/3
                correctedAngles(j,cj) = pi/3;
            end
        end
        
    end
    
    if NS ~= 0
        for j = 1:NS
            Angle_s = [Angles_s(j), Angles_s(j + NS), Angles_s(j + 2*NS), Angles_s(j + 3*NS)];
        %         if (Camera_Position(3, i) < pi/3)
            Angle_s1 = mod((Angle_s - mod((Cameras_Position(3, i) - Cameras_Position(4, i)/2), 2*pi)), 2*pi);
            a_s(j) = ~isempty(find((Angle_s1 >= 0) & (Angle_s1 <= Cameras_Position(4, i)), 1));
        %         a(j) = ~isempty(find((Angle > (Cameras_Position(3, i) - Cameras_Position(4, i)/2)) & ...,
        %                          (Angle < (Cameras_Position(3, i) + Cameras_Position(4, i)/2)), 1));
            b_s(j) = boolean(norm(obstacle_parameters(:,j) - Cameras_Position(1:2, i)) < Cameras_Position(5, i));
        
            if (abs(obstacle_parameters(1, j)-Cameras_Position(1, i)) < ObstacleSize_S/2 && abs(obstacle_parameters(2, j)-Cameras_Position(2, i)) < ObstacleSize_S/2 )
                a_s(j) = 1;
            end
        
            Distances_s(j, i) = norm(obstacle_parameters(:,j) - Cameras_Position(1:2, i));
        end
    else
        Distances_s = [];
        
    end
    
    %% output
    c = a.*b;
    index = c == 1;   
    
    Table(i, index) = 1; % FOV table
    
    if NS ~= 0
        c_s = a_s.*b_s;
        index_s = c_s == 1;
        Table_s(i, index_s) = 1;
    
    
    
        index_1 = [Distances(:, i)', Distances_s(:, i)'; 1:NO, 1:NS; ones(1, NO), 2*ones(1, NS)];
        index_2 = sortrows(index_1')';
    
        range_total = [0, 0];
        Oin = 0;
        for ii = 1:NO+NS
            if (index_2(3, ii) == 1)
                jj1 = index_2(2, ii);
                if (Table(i, jj1) == 1)
                    Oin = Oin + 1;
                    if (abs(Objects_Position(1, jj1)-Cameras_Position(1, i)) < ObjectSize/2 && abs(Objects_Position(2, jj1)-Cameras_Position(2, i)) < ObjectSize/2 )
                        occRate(i, jj1) = 1;
                    else
                        Angle2 = [Angles(jj1), Angles(jj1 + NO), Angles(jj1 + 2*NO), Angles(jj1 + 3*NO)]; 
                        Angle3 = mod((Angle2 - mod((Cameras_Position(3, i) - Cameras_Position(4, i)/2), 2*pi)), 2*pi);
               
                        if (max(Angle3) > pi)
                            range_min = 0;
                            if (sum(Angle3 > Cameras_Position(4, i) & Angle3 < pi) == 1)
                                range_max = Cameras_Position(4, i);
                            else
                                range_max = max(Angle3(Angle3 < Cameras_Position(4, i)));
                            end
                            range = mod(max(Angle3(Angle3 < pi)) - min(Angle3(Angle3 > pi)), 2*pi);
                        elseif (max(Angle3) > Cameras_Position(4, i))
                            range_min = min(Angle3);
                            range_max = Cameras_Position(4, i);
                            range = max(Angle3) - range_min;
                        else
                            range_min = min(Angle3);
                            range_max = max(Angle3);
                            range = range_max - range_min;
                        end
               
                        range_min_final = range_min;
                        range_max_final = range_max;
                        for ii1 = 1:Oin
                            if (range_min_final > range_total(ii1, 1) && range_min_final < range_total(ii1, 2) && range_max_final > range_total(ii1, 1) && range_max_final < range_total(ii1, 2))
                                range_min_final = 0;
                                range_max_final = 0;
                                break;
                            else
                                if (range_min_final > range_total(ii1, 1) && range_min_final < range_total(ii1, 2))
                                    range_min_final = range_total(ii1, 2);
                                end
                                if (range_max_final > range_total(ii1, 1) && range_max_final < range_total(ii1, 2))
                                    range_max_final = range_total(ii1, 1);
                                end
                            end
                       
                        end
               
               
                        occRate(i, jj1) = (range_max_final - range_min_final)/range;
                        range_total = [range_total; range_min, range_max];
                    end
                
                
                end
            else
                jj2 = index_2(2, ii);
                if (Table_s(i, jj2) == 1)
                    Oin = Oin + 1;
                    Angle_s2 = [Angles_s(jj2), Angles_s(jj2 + NS), Angles_s(jj2 + 2*NS), Angles_s(jj2 + 3*NS)]; 
                    Angle_s3 = mod((Angle_s2 - mod((Cameras_Position(3, i) - Cameras_Position(4, i)/2), 2*pi)), 2*pi);
               
                    if (max(Angle_s3) > pi)
                        range_min = 0;
                        if (sum(Angle_s3 > Cameras_Position(4, i) & Angle_s3 < pi) == 1)
                            range_max = Cameras_Position(4, i);
                        else
                            range_max = max(Angle_s3(Angle_s3 < Cameras_Position(4, i)));
                        end
                        range = mod(max(Angle_s3(Angle_s3 < pi)) - min(Angle_s3(Angle_s3 > pi)), 2*pi);
                    elseif (max(Angle_s3) > Cameras_Position(4, i))
                        range_min = min(Angle_s3);
                        range_max = Cameras_Position(4, i);
                        range = max(Angle_s3) - range_min;
                    else
                        range_min = min(Angle_s3);
                        range_max = max(Angle_s3);
                        range = range_max - range_min;
                    end
               
                    range_min_final = range_min;
                    range_max_final = range_max;
                    range_total = [range_total; range_min, range_max];
                end
            end
        end
    else
        index_1 = [Distances(:, i)'; 1:NO];
        index_2 = sortrows(index_1')';
        range_total = [0, 0];
        Oin = 0;
        for ii = 1:NO
            jj1 = index_2(2, ii);
            if (Table(i, jj1) == 1)
                Oin = Oin + 1;
                if (abs(Objects_Position(1, jj1)-Cameras_Position(1, i)) < ObjectSize/2 && abs(Objects_Position(2, jj1)-Cameras_Position(2, i)) < ObjectSize/2 )
                    occRate(i, jj1) = 1;
                else
                    Angle2 = [Angles(jj1), Angles(jj1 + NO), Angles(jj1 + 2*NO), Angles(jj1 + 3*NO)]; 
                    Angle3 = mod((Angle2 - mod((Cameras_Position(3, i) - Cameras_Position(4, i)/2), 2*pi)), 2*pi);
               
                    if (max(Angle3) > pi)
                        range_min = 0;
                        if (sum(Angle3 > Cameras_Position(4, i) & Angle3 < pi) == 1)
                            range_max = Cameras_Position(4, i);
                        else
                            range_max = max(Angle3(Angle3 < Cameras_Position(4, i)));
                        end
                        range = mod(max(Angle3(Angle3 < pi)) - min(Angle3(Angle3 > pi)), 2*pi);
                    elseif (max(Angle3) > Cameras_Position(4, i))
                        range_min = min(Angle3);
                        range_max = Cameras_Position(4, i);
                        range = max(Angle3) - range_min;
                    else
                        range_min = min(Angle3);
                        range_max = max(Angle3);
                        range = range_max - range_min;
                    end
               
                    range_min_final = range_min;
                    range_max_final = range_max;
                    for ii1 = 1:Oin
                        if (range_min_final > range_total(ii1, 1) && range_min_final < range_total(ii1, 2) && range_max_final > range_total(ii1, 1) && range_max_final < range_total(ii1, 2))
                            range_min_final = 0;
                            range_max_final = 0;
                        break;
                        else
                            if (range_min_final > range_total(ii1, 1) && range_min_final < range_total(ii1, 2))
                                range_min_final = range_total(ii1, 2);
                            end
                            if (range_max_final > range_total(ii1, 1) && range_max_final < range_total(ii1, 2))
                                range_max_final = range_total(ii1, 1);
                            end
                        end
                       
                    end
               
               
                    occRate(i, jj1) = (range_max_final - range_min_final)/range;
                    range_total = [range_total; range_min, range_max];
                end
                
                
            end
        end
        
        
%         if (index_2(3, ii) == 1)
%             jj = index_2(2, ii);
%         
%             if (Table(i, jj) == 1)
%                 Oin = Oin + 1;
%                 if (abs(Objects_Position(1, jj)-Cameras_Position(1, i)) < ObjectSize/2 && abs(Objects_Position(2, jj)-Cameras_Position(2, i)) < ObjectSize/2 )
%                     occRate(i, jj) = 1;
%                 else
%                     Angle2 = [Angles(jj), Angles(jj + NO), Angles(jj + 2*NO), Angles(jj + 3*NO)]; 
%                     Angle3 = mod((Angle2 - mod((Cameras_Position(3, i) - Cameras_Position(4, i)/2), 2*pi)), 2*pi);
%                
%                     if (max(Angle3) > pi)
%                         range_min = 0;
%                         if (sum(Angle3 > Cameras_Position(4, i) & Angle3 < pi) == 1)
%                             range_max = Cameras_Position(4, i);
%                         else
%                             range_max = max(Angle3(Angle3 < Cameras_Position(4, i)));
%                         end
%                         range = mod(max(Angle3(Angle3 < pi)) - min(Angle3(Angle3 > pi)), 2*pi);
%                     elseif (max(Angle3) > Cameras_Position(4, i))
%                         range_min = min(Angle3);
%                         range_max = Cameras_Position(4, i);
%                         range = max(Angle3) - range_min;
%                     else
%                         range_min = min(Angle3);
%                         range_max = max(Angle3);
%                         range = range_max - range_min;
%                     end
%                
%                     range_min_final = range_min;
%                     range_max_final = range_max;
%                     for ii1 = 1:Oin
%                         if (range_min_final > range_total(ii1, 1) && range_min_final < range_total(ii1, 2) && range_max_final > range_total(ii1, 1) && range_max_final < range_total(ii1, 2))
%                             range_min_final = 0;
%                             range_max_final = 0;
%                             break;
%                         else
%                             if (range_min_final > range_total(ii1, 1) && range_min_final < range_total(ii1, 2))
%                                 range_min_final = range_total(ii1, 2);
%                             end
%                             if (range_max_final > range_total(ii1, 1) && range_max_final < range_total(ii1, 2))
%                                 range_max_final = range_total(ii1, 1);
%                             end
%                         end
%                        
%                     end
%                
%                
%                     occRate(i, jj) = (range_max_final - range_min_final)/range;
%                     range_total = [range_total; range_min, range_max];
%                 end
%                
%             end
%             
%         end
    end
    
    
    
    
    
    
    
%     for ii = 1:NO
%         if c(i,ii) == 1
%             for jj = ii+1:NO
%                 if c(i,jj) == 1
%                     min1 = min(correctedAngles(ii,:));
%                     max1 = max(correctedAngles(ii,:));
%                     min2 = min(correctedAngles(jj,:));
%                     max2 = max(correctedAngles(jj,:));
%                     
%                     if max2 > min1 && min2 < max1
%                         if savedDistance(ii,1) < savedDistance(jj,1)
%                             c1(i,jj) = ii;
%                             occRate(i,jj) = (max2-min1)/(max2-max1); % to be changed
%                         else
%                             c1(i,ii) = jj;
%                             occRate(i,jj) = (max2-min1)/(max2-max1); % to be changed
%                         end
%                     end
%                     
%                     index1 = (c1 ~= 0);
%                     TableOcc(i, index1) = c1(i, index1);
%                 end
%             end
%         end
%         
%         
%         %
%         %     min1 = min(savedAngles(1,:));
%         %     max1 = max(savedAngles(1,:));
%         %     min2 = min(savedAngles(2,:));
%         %     max2 = max(savedAngles(2,:));
%         
%         %     c1 = c;
%         
%         %         if max2 > min1 && min2 < max1
%         %             if savedDistance(1,1) < savedDistance(2,1)
%         %                 c1(1,2) = 1;
%         %             else c1(1,1) = 2;
%         %             end
%         %         end
%         %
%         %         index1 = (c1 ~= 0);
%         %         TableOcc(i, index1) = c1(i, index1);
%         
%         %     index1 = find(c == 1);
%         %         if size(index1,2) >1
%         %         for k = 1 : size(index1,2)
%         %             objAngle = savedAngles(index1[k],:);
%         %             if max(objAngle) > pi
%         %             if max(objAngle) > pi/3
%         %
%         %         end
%         %     end
%         
%         
%         %     Table(i+NC,index1) = 1; % occlusion table
%         
%     end
    
end