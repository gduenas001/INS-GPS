% function convert_LIDAR_data_from_txt_to_mat(num_of_LIDAR_frames)

num_of_LIDAR_frames= 3092
for i=0:(num_of_LIDAR_frames-1)
    
    % read the text file
    fileID= fopen(['textFiles/',num2str(i),'.txt'],'r');
    z= transpose(fscanf(fileID,'%f',[2,inf]));
    fclose(fileID);
    % TODO: this two tranformations will be done in the ROS code directly
    if ~isempty(z)
        z= [-(z(:,2)+0.335),-(z(:,1))]; % for the vehicle dataset
    end
    
    % save as mat files
    save(['matFiles/','Epoch',num2str(i),'.mat'],'z');
end
