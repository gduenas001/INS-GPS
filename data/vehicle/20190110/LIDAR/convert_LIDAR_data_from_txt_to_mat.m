function convert_LIDAR_data_from_txt_to_mat(num_of_LIDAR_frames)

for i=0:num_of_LIDAR_frames-1
    
    % read the text file
    fileID= fopen(['textFiles/','Epoch',num2str(i),'.txt'],'r');
    z= transpose(fscanf(fileID,'%f',[2,inf]));
    
    % TODO: this two tranformations will be done in the ROS code directly
    if ~isempty(z)
        z= [-z(:,2),z(:,1)];
        z= [-z(:,2),-z(:,1)];
    end
    
    % save as mat files
    save(['matFiles/','Epoch',num2str(i),'.mat'],'z');
end
