function Plot_Mode_Probability_7(track_histories, track_ID)
    % Plot IMM mode probability evolution for a specific hardware track
    
    if track_ID > numel(track_histories) || isempty(track_histories{track_ID})
        fprintf('Track %d does not exist or is empty.\n', track_ID);
        return;
    end
    
    track_history = track_histories{track_ID};
    
    track_time = track_history(:, 1);
    mu_cv = track_history(:, 11);
    mu_ca = track_history(:, 12);
    
    figure('Name', sprintf('Track %d: IMM Mode Probabilities', track_ID), ...
           'Position', [100 100 900 350]);
    
    area(track_time, [mu_cv, mu_ca]);
    colororder([0.2 0.6 1.0; 1.0 0.3 0.2]);
    
    ylabel('Mode Probability');
    xlabel('Time (s)');
    legend('CV', 'CA', 'Location', 'best');
    title(sprintf('Track %d | IMM Mode Probability Evolution | Duration: %.1fs', ...
                  track_ID, track_time(end) - track_time(1)));
    ylim([0 1]);
    xlim([track_time(1) track_time(end)]);
    grid on;
end