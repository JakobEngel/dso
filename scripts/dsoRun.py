#
#	author: Michal Nowicki
#
from subprocess import call
import subprocess
import sys
import glob
import os

# Path to save main results - create if needed
if not os.path.exists("results"):
	os.makedirs("results");	
else:
	call('rm results/*', shell=True);


sequences = [
'01', \
'02', \
'03', \
'04', \
'05', \
'06', \
'07', \
'08', \
'09', \
'10', \
'11', \
'12', \
'13', \
'14', \
'15', \
'16', \
'17', \
'18', \
'19', \
'20', \
'21', \
'22', \
'23', \
'24', \
'25', \
'26', \
'27', \
'28', \
'29', \
'30', \
'31', \
'32', \
'33', \
'34', \
'35', \
'36', \
'37', \
'38', \
'39', \
'40', \
'41', \
'42', \
'43', \
'44', \
'45', \
'46', \
'47', \
'48', \
'49', \
'50'];

runsPerSequence = 1;

if len(sys.argv) != 2:
	print('Please provide the name of the host machine, e.g. python orbslamRun.py lrm2')
else:

	mainDatasetPath = '';

	# PPCM
	if "ppcm" in sys.argv[1]:
		mainDatasetPath = '/home/michal/dsoDataset';

	# LRM2
	if "lrm2" in sys.argv[1]:
		mainDatasetPath = '/mnt/data/Datasets/dso';

	# Toshiba Portege Z30
	if "local" in sys.argv[1]:
		mainDatasetPath = '/media/michalnowicki/MNowicki-Private/DSO/sequences';

	print 'mainDatasetPath: ' + mainDatasetPath


	detectionTypes = [2, 5, 7]#[7, 7, 7, 7]; #[6, 6, 6, 6, 6, 6] #[4, 4]#[] 1, 1, 1];
	detectionTypeFastThreshold = [2]#[0, 0, 0, 0, 0]; #[7, 2];#[0, 15, 10, 5];
	harrisK = [0]#[0.01, 0.01, 0.01, 0.01];#[0.002, 0.005, 0.01, 0.02, 0.04, 0.08];
	lambdaThreshold = [0.0000001]#[0.001, 0.0001, 0.00001, 0.000001];

	# Clear the results directories
	for (det, lamb) in zip(detectionTypes, lambdaThreshold):
		if not os.path.exists("results/det_"+str(det)+"_"+str(lamb)):
			os.makedirs("results/det_"+str(det)+"_"+str(lamb));
		else:
			call('rm results/det_'+str(det)+'_'+str(lamb) +'/*', shell=True);

	for seq in sequences:

		for (det, fastThr, hark, lamb) in zip(detectionTypes, detectionTypeFastThreshold, harrisK, lambdaThreshold):

			for runId in range(0, runsPerSequence):
				print("Current sequence: " + seq);

				print('./dso_dataset files='+mainDatasetPath+'/sequence_'+ seq +'/images.zip calib='+mainDatasetPath+'/sequence_'+ seq +'/camera.txt gamma='+mainDatasetPath+'/sequence_'+ seq +'/pcalib.txt vignette='+mainDatasetPath+'/sequence_'+ seq +'/vignette.png preset=0 mode=0 nogui=1 reverse=0 quiet=1 detectionType=' + str(det) + ' detectionTypeFastThreshold=' +str(fastThr) +' harrisK=' + str(hark) +' lambdaThreshold=' + str(lamb));

				# Copy to currently used settings
				call('./dso_dataset files='+mainDatasetPath+'/sequence_'+ seq +'/images.zip calib='+mainDatasetPath+'/sequence_'+ seq +'/camera.txt gamma='+mainDatasetPath+'/sequence_'+ seq +'/pcalib.txt vignette='+mainDatasetPath+'/sequence_'+ seq +'/vignette.png preset=0 mode=0 nogui=1 reverse=0 quiet=1 detectionType=' + str(det) + ' detectionTypeFastThreshold=' +str(fastThr) +' harrisK=' + str(hark) +' lambdaThreshold=' + str(lamb), shell=True);

				# Run software
				call('mv result.txt results/det_'+str(det)+'_'+str(lamb) +'/sequence_' + str(seq) + '_' + str(runId) + '.txt', shell=True);

