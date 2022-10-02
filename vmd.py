#By Minmode

from inc_noesis import *
import os.path
import io
import math
import shlex

class Vmd:

    def __init__(self, name, frameCount, animData, morphData, boneDictName, morphDictName, pmxScale, scaleExcludeList):
        self.name = name
        self.frameCount = frameCount
        self.animData = animData
        self.morphData = morphData
        self.boneDictName = boneDictName
        self.morphDictName = morphDictName
        self.pmxScale = pmxScale
        self.scaleExcludeList = scaleExcludeList

    def wrtieVmd(self, outDir):
        writeName = outDir + self.name + ".vmd"
        f = open(writeName, 'wb')
        f.write("Vocaloid Motion Data 0002".encode('shift_jis').ljust(30, b'\0'))#magic
        f.write(self.name.encode('shift_jis')[:19].ljust(20, b'\0'))#model name
        self.writeAnim(f)
        self.writeMorph(f)
        f.write(struct.pack('4i',0,0,0,0))
        f.close()
        print("Successfully wrote " + self.name + ".vmd")

    def writeAnim(self, f):
        if self.boneDictName:
            boneDict = self.readDict(self.boneDictName)
        else:
            boneDict = {}
        keyFrames = []
        anim = self.animData
        for i in range(len(anim.kfBones)):
            kfBone = anim.kfBones[i]
            name = anim.bones[kfBone.boneIndex].name
            posFrames = kfBone.translationKeys
            rotFrames = kfBone.rotationKeys
            sclFrames = kfBone.scaleKeys
            if posFrames == []:
                posFrames = [NoeKeyFramedValue(0, NoeVec3())]
            if rotFrames == []:
                rotFrames = [NoeKeyFramedValue(0, NoeQuat())]
            if sclFrames == []:
                sclFrames = [NoeKeyFramedValue(0, NoeVec3((1.0, 1.0, 1.0)))]
            posIdx = 0
            rotIdx = 0
            sclIdx = 0
            prePos = posFrames[0].value
            preRot = rotFrames[0].value
            preScl = sclFrames[0].value
            wScl = NoeVec3((1.0, 1.0, 1.0))
            for a in range(self.frameCount):
                if a == posFrames[posIdx].time or a == rotFrames[rotIdx].time:
                    keyFrame = KeyFrame(name, a)
                    if len(posFrames) == 1 and posFrames[0].value == NoeVec3():
                        keyFrame.pos = prePos
                    elif a == posFrames[posIdx].time:
                        pos = posFrames[posIdx].value
                        if anim.bones[kfBone.boneIndex].parentIndex != -1:
                            pMatrix = anim.bones[anim.bones[kfBone.boneIndex].parentIndex]._matrix
                            matrix = anim.bones[kfBone.boneIndex]._matrix
                            bm = pMatrix.toQuat().toMat43().inverse()
                            bms = math.sqrt((pMatrix[0][0] * pMatrix[0][0]) + (pMatrix[1][0] * pMatrix[1][0]) + (pMatrix[2][0] * pMatrix[2][0]))
                            bm[3][0] = pos[0]
                            bm[3][1] = pos[1]
                            bm[3][2] = pos[2]
                            bm = bm.inverse()
                            posX = (bm[3][0] * -1 * bms * wScl[0]) - (matrix[3][0] - pMatrix[3][0])
                            posY = (bm[3][1] * -1 * bms * wScl[1]) - (matrix[3][1] - pMatrix[3][1])
                            posZ = (bm[3][2] * -1 * bms * wScl[2]) - (matrix[3][2] - pMatrix[3][2])
                            pos = NoeVec3((posX, posY, posZ))
                        keyFrame.pos = pos
                        prePos = pos
                        if posIdx + 1 != len(posFrames):
                            posIdx += 1
                    else:
                        keyFrame.pos = prePos
                    if len(rotFrames) == 1 and rotFrames[0].value == NoeQuat():
                        keyFrame.rot = preRot
                    elif a == rotFrames[rotIdx].time:
                        rot = rotFrames[rotIdx].value
                        matrix = anim.bones[kfBone.boneIndex]._matrix.toQuat()
                        if anim.bones[kfBone.boneIndex].parentIndex != -1:
                            pMatrix = anim.bones[anim.bones[kfBone.boneIndex].parentIndex]._matrix.toQuat()
                            rot *= pMatrix
                        matrix[0] *= -1
                        matrix[1] *= -1
                        matrix[3] *= -1
                        rot[0] *= -1
                        rot[1] *= -1
                        rot[3] *= -1
                        rot = (rot.toMat43() * matrix.toMat43().inverse()).toQuat()
                        keyFrame.rot = rot
                        preRot = rot
                        if rotIdx + 1 != len(rotFrames):
                            rotIdx += 1
                    else:
                        keyFrame.rot = preRot
                    keyFrames.append(keyFrame)
        f.write(struct.pack('i', len(keyFrames)))
        for keyFrame in keyFrames:
            if keyFrame.name in boneDict:
                f.write(boneDict[keyFrame.name].encode('shift_jis')[:14].ljust(15, b'\0'))
            else:
                f.write(keyFrame.name.encode('shift_jis')[:14].ljust(15, b'\0'))
            f.write(struct.pack('i', keyFrame.frame))
            if keyFrame.name not in self.scaleExcludeList:
                f.write(struct.pack('f', keyFrame.pos[0] * self.pmxScale))
                f.write(struct.pack('f', keyFrame.pos[1] * self.pmxScale))
                f.write(struct.pack('f', keyFrame.pos[2] * -1 * self.pmxScale))
            else:
                f.write(struct.pack('f', keyFrame.pos[0]))
                f.write(struct.pack('f', keyFrame.pos[1]))
                f.write(struct.pack('f', keyFrame.pos[2]))
            f.write(keyFrame.rot.toBytes())
            f.write(bytearray.fromhex("14 14 00 00 14 14 14 14 6B 6B 6B 6B 6B 6B 6B 6B 14 14 14 14 14 14 14 6B 6B 6B 6B 6B 6B 6B 6B 00 14 14 14 14 14 14 6B 6B 6B 6B 6B 6B 6B 6B 00 00 14 14 14 14 14 6B 6B 6B 6B 6B 6B 6B 6B 00 00 00"))

    def writeMorph(self, f):
        if self.morphDictName:
            morphDict = self.readDict(self.morphDictName)
        else:
            morphDict = {}
        f.write(struct.pack('i', len(self.morphData)))
        for morph in self.morphData:
            if morph.name in morphDict:
                f.write(morphDict[morph.name].encode('shift_jis')[:14].ljust(15, b'\0'))
            else:
                f.write(morph.name.encode('shift_jis')[:14].ljust(15, b'\0'))
            f.write(struct.pack('i', morph.frame))
            f.write(struct.pack('f', morph.blend))

    def readDict(self, dictName):
        d = {}
        with io.open(os.getcwd() + "/dicts/" + dictName, mode="r", encoding="utf-8") as f:
            for line in f:
                if not line.startswith("#"):
                    (key, val) = shlex.split(line)
                    d[key] = val
        return d

class KeyFrame:

    def __init__(self, name, frame):
        self.name = name
        self.frame = frame
        self.pos = None
        self.rot = None