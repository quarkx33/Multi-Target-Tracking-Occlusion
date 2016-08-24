function r = plot_anim(x,est, measr, obj_idx_occlude, all_particles, global_vars,  eff_sample_size, relative_probs, models_to_consider)

close all;
x_pos = x(1:3:end,:); % plot only x-coordinates
est = est(1:3:end,:); 

num_objects = size(x_pos,1);

figure('position',[10, 50,1500, 720]);
subplot(2,9,10);
set(gca,'Position',[0.02 0.05 0.95 0.45]);
width_shift= 0.95/9;
eps = 0.04;

ax = gca;

%ax.XScale = 'log';

%color_map =['red'
%h = hgtransform('Parent',ax);
%hold on;
%plot(x(:,1),ones(size(x,1),1),'o','MarkerFaceColor','red','Parent',h);
%hold off;

%text(x(:,1),ones(size(x,1),1),num2str([1:size(x,1)]')); %,'Parent',h, 'VerticalAlignment','top','FontSize',10);

ax.XLimMode = 'manual';
ax.YLimMode = 'manual';

%ax.XLim = [0, x(1,end)];
%ax.YLim = [0.5, 1.5];

%ax.XLim = [0, 200];
drawnow;




for ii = 2:size(x_pos,2)
    clf;
    %m = makehgtform('translate',x(1,ii)-x(1,1),0,0);
    %h.Matrix = m;
    %t.String = num2str(y(k));
    %h = hgtransform('Parent',ax);
    subplot(2,9,10);
    ax = gca;
    set(gca,'Position',[0.02 0.05 0.95 0.45]);
    plot(x_pos(1,ii),1,'o','MarkerFaceColor','black', 'MarkerSize',5); 
    hold on;
    text(x_pos(1,ii),1,num2str(1), 'VerticalAlignment','top','FontSize',5);
    
    plot(est(1,ii),1.2,'o','MarkerFaceColor','blue', 'MarkerSize',5); 
    text(est(1,ii),1.2,num2str(1), 'VerticalAlignment','top','FontSize',5);
    
    
    if obj_idx_occlude(1,ii)
        %object is NOT occluded
        plot(measr(1,ii),1.1,'o','MarkerFaceColor','green', 'MarkerSize',5);
    else
        plot(measr(1,ii),1.1,'o','MarkerFaceColor','red', 'MarkerSize',5);    
    end
    text(measr(1,ii),1.1,num2str(1), 'VerticalAlignment','top','FontSize',5);
    
    legend('Actual', 'Estimated', 'Measurement', 'Location','northwest');
    
    plot(est(1,ii),1.2,'o','MarkerFaceColor','blue', 'MarkerSize',5); 
    text(est(1,ii),1.2,num2str(1), 'VerticalAlignment','top','FontSize',5);
    
    

    for jj = 2: num_objects
        plot(x_pos(jj,ii),1,'o','MarkerFaceColor','black', 'MarkerSize',5); %,'Parent',h);
        text(x_pos(jj,ii),1,num2str(jj), 'VerticalAlignment','top','FontSize',5); %,'Parent',h, 'VerticalAlignment','top','FontSize',10);

        plot(est(jj,ii),1.2,'o','MarkerFaceColor','blue', 'MarkerSize',5); 
        text(est(jj,ii),1.2,num2str(jj), 'VerticalAlignment','top','FontSize',5);

        if obj_idx_occlude(jj,ii)
            plot(measr(jj,ii),1.1,'o','MarkerFaceColor','green', 'MarkerSize',5);
        else
            plot(measr(jj,ii),1.1,'o','MarkerFaceColor','red', 'MarkerSize',5);
        end
        text(measr(jj,ii),1.1,num2str(jj), 'VerticalAlignment','top','FontSize',5);
    
 
    end
    %ax.XScale = 'log';
    ax.XLim = [0, x(1,end)];
    ax.YLim = [0.5, 1.5];

    
    for iter = 1: size(global_vars.occlusion_blocs_cord,1)
        a = histogram(global_vars.occlusion_blocs_cord(iter,:),1, 'EdgeColor', 'none', 'FaceColor','b');
        alpha(a,0.3); %Set the transparency of the occlusion band
        
        %plot(repmat(global_vars.occlusion_blocs_cord(iter,1),2),[.7, 1.5],'r-','LineWidth',2);
% %         tmp = strcat(num2str(global_vars.occlusion_blocs_cord(iter,1)) , '-->');
% %         text(global_vars.occlusion_blocs_cord(iter,1),0.5,tmp, 'VerticalAlignment','top','FontSize',10);
% %         
% %         %plot(repmat(global_vars.occlusion_blocs_cord(iter,2),2),[.7, 1.5],'r-','LineWidth',2);
% %         tmp = strcat('<--', num2str(global_vars.occlusion_blocs_cord(iter,2)));
% %         text(global_vars.occlusion_blocs_cord(iter,2),0.5,tmp, 'VerticalAlignment','top','HorizontalAlignment', 'right','FontSize',10);

    end
        
        
    %ax.XLim = [0, 500];
    
    hold off;
    
        
    
    
    
    % ===========now plot the histograms for all state vectors---

    subplot(2,9,1);
    xlabel('x-position', 'FontSize', 8); title({'car 1 X-pos vs. measr'}, 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05 0.5838 width_shift-eps 0.3412])
    %mean_pos = mean(all_particles(1,:,ii));
    h = histogram(all_particles(1,:,ii), 10, 'Normalization','probability', 'FaceColor','b', 'EdgeColor', 'none');
    %plot(repmat(measr(1,ii),2),[0,max(h.Values)],'r-', 'LineWidth',2); 
    plot(repmat(measr(1,ii),2),[0,max(h.Values)],'r-', 'LineWidth',2); 
    hold off;
    
    
    subplot(2,9,2);
    xlabel('x-velocity', 'FontSize', 8); title('car 1 X-vel vs. actual', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05 + width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(2,:,ii), 10, 'Normalization','probability','FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(x(2,ii),2),[0,max(h.Values)],'g-', 'LineWidth',2);
%     ax = gca;
%     ax.XLim = [temporal_params.mean_target_vel-3,temporal_params.mean_target_vel+3];
%     ax.XLim = [0, global_vars.v_max];
    hold off;
    
    subplot(2,9,3);
    xlabel('x-accleration', 'FontSize', 8); title('car 1 X-accln vs. actual', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 2*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(3,:,ii), 10, 'Normalization','probability', 'FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(x(3,ii),2),[0,max(h.Values)],'g-', 'LineWidth',2);
    hold off;
    
    
    
    
    % car 2---
    subplot(2,9,4);
    xlabel('x-position', 'FontSize', 8); title('car 2 X-pos vs. measr', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 3*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(4,:,ii),10, 'Normalization','probability', 'FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(measr(2,ii),2),[0,max(h.Values)],'r-', 'LineWidth',2);
    hold off;
    
    
    subplot(2,9,5);
    xlabel('x-velocity', 'FontSize', 8); title('car 2 X-vel vs. actual', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 4*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(5,:,ii), 10, 'Normalization','probability','FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(x(5,ii),2),[0,max(h.Values)],'g-', 'LineWidth',2);
    hold off;
    
    subplot(2,9,6);
    xlabel('x-accleration', 'FontSize', 8); title('car 2 X-accln vs. actual', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 5*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(6,:,ii), 10, 'Normalization','probability','FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(x(6,ii),2),[0,max(h.Values)],'g-', 'LineWidth',2);
    hold off;

    
    
    %car 3---
     subplot(2,9,7);
    xlabel('x-position', 'FontSize', 8); title('car 3 X-pos vs. measr', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 6*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(7,:,ii), 10, 'Normalization','probability','FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(measr(3,ii),2),[0,max(h.Values)],'r-', 'LineWidth',2);
    hold off;
    
    
   
    
    subplot(2,9,9);
    xlabel('x-accleration', 'FontSize', 8); title('car 3 X-accln vs. actual', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 8*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(9,:,ii), 10, 'Normalization','probability', 'FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(x(9,ii),2),[0,max(h.Values)],'g-', 'LineWidth',2);
    hold off;
    
    
     subplot(2,9,8);
    xlabel('x-velocity', 'FontSize', 8); title('car 3 X-vel vs. actual', 'FontSize', 8);
    hold on;
    set(gca,'Position',[0.05+ 7*width_shift 0.5838 width_shift-eps 0.3412])
    h = histogram(all_particles(8,:,ii), 10, 'Normalization','probability', 'FaceColor','b', 'EdgeColor', 'none');
    plot(repmat(x(8,ii),2),[0,max(h.Values)],'g-', 'LineWidth',2);
    hold off;
    
    
 
    
    %p.XData = x(:,ii);
    %ax.XLim = [max(0, min(x(:,ii))-30), max(x(:,ii))+30];
    drawnow;
    %pause(0.5);
    
end



    
r=0;





