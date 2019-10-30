function back = walay( picture )
 Agray = rgb2gray(picture);
     imageSize = size(picture);
     numRows = imageSize(1);
    numCols = imageSize(2);


       wavelengthMin = 4/sqrt(2);
    wavelengthMax = hypot(numRows,numCols);
    n = floor(log2(wavelengthMax/wavelengthMin));
    wavelength = 2.^(0:(n-2)) * wavelengthMin;

    deltaTheta = 45;
    orientation = 0:deltaTheta:(180-deltaTheta);

    g = gabor(wavelength,orientation);
    gabormag = imgaborfilt(Agray,g);
    parfor i = 1:length(g)
    sigma = 0.5*g(i).Wavelength;
    K = 3;
    gabormag(:,:,i) = imgaussfilt(gabormag(:,:,i),K*sigma); 
    end
    X = 1:numCols;
    Y = 1:numRows;
    [X,Y] = meshgrid(X,Y);
    featureSet = cat(3,gabormag,X);
    featureSet = cat(3,featureSet,Y);
    numPoints = numRows*numCols;
    X = reshape(featureSet,numRows*numCols,[]);
    X = bsxfun(@minus, X, mean(X));
    X = bsxfun(@rdivide,X,std(X));
    coeff = pca(X);
    feature2DImage = reshape(X*coeff(:,1),numRows,numCols);

    L = kmeans(X,2,'Replicates',5);
	L = reshape(L,[numRows numCols]);

    Aseg1 = zeros(size(picture),'like',picture);
%    Aseg2 = zeros(size(picture),'like',picture);
    BW = L == 2;
    BW = repmat(BW,[1 1 3]);
    Aseg1(BW) = picture(BW);
%      Aseg2(~BW) = picture(~BW);

%     foreground = bwareaopen(Aseg1(BW), 8);
    labelimage = logical(Aseg1(BW), 8);
    measurements = regionprops(labelimage, 'BoundingBox');
   
    
    imshow(Aseg1);
    hold on 
    parfor object = 1:length(measurements)
        cc = measurements(object).BoundingBox;
     
    end
    hold off
    drawnow;


end

