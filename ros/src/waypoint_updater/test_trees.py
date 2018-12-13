from scipy.spatial import KDTree
from scipy.spatial import cKDTree
import time
import random
import pandas

# Track map dataframe
Data = pandas.read_csv('the_track_map_export.txt', sep=",", header=None)

# X points of track map
Xs = Data[0].values
# Y points of track map
Ys = Data[1].values

# Tree options
K = cKDTree([[x, y] for x, y in zip(Xs, Ys)])
G = KDTree([[x, y] for x, y in zip(Xs, Ys)])

count_cDKTree_faster_total = 0
count_cDKTree_faster_worst = 0

attempt_max_number = 10
queries_per_attempt = len(Xs)

f = open('test_points.txt', 'w')

for attempt in range(attempt_max_number):
	print "Attempt Number: ", attempt + 1
	sum1 = 0;
	max1 = 0;
	sum2 = 0;
	max2 = 0;
	for idx in range(queries_per_attempt):
		random.seed(idx)
	
		# Apply a random offset to the track map point
		# Test poiints +/- 50m from the track point in the x, y directions.
		x_test = Xs[idx] + 100 * (random.random() - 0.5)
		y_test = Ys[idx] + 100 * (random.random() - 0.5)
		
		if attempt == 1:
			# Export the test points used for the first check
			f.write(str(x_test))
			f.write(",")
			f.write(str(y_test))
			f.write("\n")
		
		start1 = time.time()
		A = K.query((x_test, y_test))
		end1 = time.time()
		sum1 += end1 - start1
		if (end1 - start1) > max1:
			max1 = end1 - start1

		start2 = time.time()
		B = G.query((x_test, y_test))
		end2 = time.time()
		sum2 += end2 - start2
		if (end2 - start2) > max2:
			max2 = end2 - start2
			
	if max1 < max2:
		count_cDKTree_faster_worst +=1
	if sum1 < sum2:
		count_cDKTree_faster_total +=1
		
	print "On average, cKDTree performed queries in ", 100 *sum1 / sum2, "% the time which was taken by KDTree" 

f.close()	
# Print conclusion to screen
print "Over ", attempt_max_number, " attempts of ", queries_per_attempt, " queries per attempt, cKDTree had quicker average query ",  100*count_cDKTree_faster_total/attempt_max_number,  "% of times"
print "Over ", attempt_max_number, " attempts of ", queries_per_attempt, " queries per attempt, cKDTree had quicker 'worst case' query ",  100*count_cDKTree_faster_worst/attempt_max_number,  "% of times"