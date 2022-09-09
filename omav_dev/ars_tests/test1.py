prevtime = 0
sample = 0

def gettime(time1):
    global prevtime, sample
    sample = time1 - prevtime
    prevtime = time1
    print("PrevTime")
    print(prevtime)
    return(sample)
