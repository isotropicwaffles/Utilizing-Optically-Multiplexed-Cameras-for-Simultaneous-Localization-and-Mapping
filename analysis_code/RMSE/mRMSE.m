function rmse = mRMSE(T, X)

x_rmse = sqrt(nanmean((interp1(T(:,1), T(:,2), X(:,1)) - X(:,2))) .^ 2);
y_rmse = sqrt(nanmean((interp1(T(:,1), T(:,3), X(:,1)) - X(:,3))) .^ 2);
z_rmse = sqrt(nanmean((interp1(T(:,1), T(:,4), X(:,1)) - X(:,4))) .^ 2);
rmse = sqrt(x_rmse.^ 2 + y_rmse.^ 2 + z_rmse.^ 2);

end