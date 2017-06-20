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


sequences = ['01', \
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


mainDatasetPath = '/mnt/data/Datasets/dso';

detectionTypes = [0, 1, 1, 1, 1];
detectionTypeFastThreshold = [-1, 20, 15, 10, 5];

# Clear the results directories
for (det, detTh) in zip(detecitonTypes, detectionTypeFastThreshold):
	if not os.path.exists("results/det_"+str(det)+"_"+str(detTh)):
		os.makedirs("results/det_"+str(det)+"_"+str(detTh));	
	else:
		call('rm results/det_'+str(det)+'_'+str(detTh) +'/*', shell=True);

for seq in sequences:

	for (det, detTh) in zip(detectionTypes, detectionTypeFastThreshold):
		
		for runId in range(0, runsPerSequence):
			print("Current sequence: " + seq);
	
			# Copy to currently used settings
			call('./dso_dataset files='+mainDatasetPath+'/sequence_'+ seq +'/images.zip calib='+mainDatasetPath+'/sequence_'+ seq +'/camera.txt gamma='+mainDatasetPath+'/sequence_'+ seq +'/pcalib.txt vignette='+mainDatasetPath+'/sequence_'+ seq +'/vignette.png preset=0 mode=0 nogui=1 reverse=0 quiet=1 detectionType=' + str(det) + ' detectionTypeFastThreshold=' + str(detTh), shell=True);	
		
			# Run software
			call('mv result.txt results/det_'+str(det)+'_'+str(detTh) +'/sequence_' + str(seq) + '_' + str(runId) + '.txt', shell=True);

