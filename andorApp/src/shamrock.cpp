/*
 * shamrock.cpp
 *
 * This is an EPICS driver for Andor Shamrock spectrographs
 *
 * Author: Mark Rivers
 *         Univerity of Chicago
 *
 * Created: April 9, 2014
 *
 * Modified: March 16, 2015 to add Flipper functions
 *
 */

#include <iocsh.h>
#include <stdlib.h>
#include <string.h>

#include <epicsString.h>

#include <asynPortDriver.h>

#include <ShamrockCIF.h>

#include <epicsExport.h>

static const char *driverName = "shamrock";


/* Shamrock driver specific parameters */
#define SRSerialNumberString            "SR_SERIAL_NUMBER"
#define SRWavelengthString              "SR_WAVELENGTH"
#define SRMinWavelengthString           "SR_MIN_WAVELENGTH"
#define SRMaxWavelengthString           "SR_MAX_WAVELENGTH"
#define SRCalibrationString             "SR_CALIBRATION"
#define SRGratingString                 "SR_GRATING"
#define SRNumGratingsString             "SR_NUM_GRATINGS"
#define SRGratingExistsString           "SR_GRATING_EXISTS"
#define SRMinGratingWavelengthString    "SR_MIN_GRATE_WAV"
#define SRMaxGratingWavelengthString    "SR_MAX_GRATE_WAV"
#define SRFlipperMirrorExistsString     "SR_FLIPPER_MIRROR_EXISTS"
#define SRFlipperMirrorPortString       "SR_FLIPPER_MIRROR_PORT"
#define SRSlitExistsString              "SR_SLIT_EXISTS"
#define SRSlitSizeString                "SR_SLIT_SIZE"
#define SRCamSensorWidthString          "SR_CAM_SENSOR_WIDTH"
#define SRCamPixelWidthString           "SR_CAM_PIXEL_WIDTH"
#define SRMinCamWavelengthString        "SR_MIN_CAM_WAVELENGTH"
#define SRMaxCamWavelengthString        "SR_MAX_CAM_WAVELENGTH"

// based on existing available Andor spectrograph models
#define MAX_GRATINGS 4 // Kymera 328i has quad grating turret

// Maximum number of address.
#define MAX_ADDR 5

/** Driver for Andor Shamrock spectrographs.
 * One instance of this class will control one spectrograph.
 */
class shamrock : public asynPortDriver
{
public:
    shamrock(const char *portName, int shamrockID, const char *iniPath, int priority, int stackSize);

    /* virtual methods to override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat32Array(asynUser *pasynUser, epicsFloat32 *pValue, size_t nElements, size_t *nIn);
    void report(FILE *fp, int details);
    asynStatus updateInitialPVs();

protected:
    int SRSerialNumber_;         /** Serial number           (octet read)        */
    #define FIRST_SR_PARAM SRSerialNumber_
    int SRWavelength_;          /** Wavelength              (float64 read/write)*/
    int SRMinWavelength_;       /** Min wavelength          (float64 read/write)*/
    int SRMaxWavelength_;       /** Min wavelength          (float64 read/write)*/
    int SRCalibration_;         /** Calibration             (float32 array read)*/
    int SRGrating_;             /** Grating                 (int32 read/write)  */
    int SRNumGratings_;         /** Number of gratings      (int32 read)        */
    int SRGratingExists_;       /** Grating exists          (int32 read)        */
    int SRMinGratingWavelength_;/** Min wavelength          (float64 read/write)*/
    int SRMaxGratingWavelength_;/** Min wavelength          (float64 read/write)*/
    int SRFlipperMirrorExists_; /** Flipper Mirror exists   (int32 read)        */
    int SRFlipperMirrorPort_;   /** Flipper Mirror Port     (int32 read/write)  */
    int SRSlitExists_;          /** Slit exists             (int32 read)        */
    int SRSlitSize_;            /** Slit width              (float64 read/write)*/
    int SRCamSensorWidth_;      /** Width of camera sensor  (int32 read/write)  */
    int SRCamPixelWidth_;       /** Width of camera pixel   (float64 read/write)*/
    int SRMinCamWavelength_;    /** wavelength of camera    (float64 read)      */
    int SRMaxCamWavelength_;    /** wavelength of camera    (float64 read)      */
    #define LAST_SR_PARAM SRMaxCamWavelength_

private:
    /* Local methods to this class */
    inline asynStatus checkError(int status, const char *functionName, const char *shamrockFunction);
    asynStatus getStatus();
    asynStatus calibrate(int port);

    /* Data */
    int shamrockId_;
    bool slitIsPresent_[SHAMROCK_SLIT_INDEX_MAX];
    int numPixels_;
    float *calibration_;
    char lastError_[SHAMROCK_ERRORLENGTH];
    bool flipperMirrorIsPresent_[SHAMROCK_FLIPPER_INDEX_MAX];
};

/** Configuration function to configure one spectrograph.
 *
 * This function need to be called once for each spectrography to be used by the IOC. A call to this
 * function instanciates one object from the shamrock class.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] shamrockId The spectrograph index.
 * \param[in] iniPath The path to the camera ini file
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int shamrockConfig(const char *portName, int shamrockId, const char *iniPath, int priority, int stackSize)
{
    shamrock *drvPvt = new shamrock(portName, shamrockId, iniPath, priority, stackSize);
    int status = drvPvt->updateInitialPVs();
    return status;
}

/** Constructor for the shamrock class
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] shamrockID The spectrograph index.
 * \param[in] iniPath The path to the camera ini file
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
shamrock::shamrock(const char *portName, int shamrockID, const char *iniPath, int priority, int stackSize)
    : asynPortDriver(portName, MAX_ADDR,
            asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask | asynDrvUserMask,
            asynInt32Mask | asynFloat64Mask | asynFloat32ArrayMask | asynOctetMask ,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),
    shamrockId_(shamrockID)
{
    static const char *functionName = "shamrock";
    asynStatus status;
    int error;
    int numDevices;

    createParam(SRSerialNumberString,           asynParamOctet,         &SRSerialNumber_);
    createParam(SRWavelengthString,             asynParamFloat64,       &SRWavelength_);
    createParam(SRMinWavelengthString,          asynParamFloat64,       &SRMinWavelength_);
    createParam(SRMaxWavelengthString,          asynParamFloat64,       &SRMaxWavelength_);
    createParam(SRCalibrationString,            asynParamFloat32Array,  &SRCalibration_);
    createParam(SRGratingString,                asynParamInt32,         &SRGrating_);
    createParam(SRNumGratingsString,            asynParamInt32,         &SRNumGratings_);
    createParam(SRGratingExistsString,          asynParamInt32,         &SRGratingExists_);
    createParam(SRMinGratingWavelengthString,   asynParamFloat64,       &SRMinGratingWavelength_);
    createParam(SRMaxGratingWavelengthString,   asynParamFloat64,       &SRMaxGratingWavelength_);
    createParam(SRFlipperMirrorPortString,      asynParamInt32,         &SRFlipperMirrorPort_);
    createParam(SRFlipperMirrorExistsString,    asynParamInt32,         &SRFlipperMirrorExists_);
    createParam(SRSlitExistsString,             asynParamInt32,         &SRSlitExists_);
    createParam(SRSlitSizeString,               asynParamFloat64,       &SRSlitSize_);
    createParam(SRCamSensorWidthString,         asynParamInt32,         &SRCamSensorWidth_);
    createParam(SRCamPixelWidthString,          asynParamFloat64,       &SRCamPixelWidth_);
    createParam(SRMinCamWavelengthString,       asynParamFloat64,       &SRMinCamWavelength_);
    createParam(SRMaxCamWavelengthString,       asynParamFloat64,       &SRMaxCamWavelength_);

    error = ShamrockInitialize((char *)iniPath);

    status = checkError(error, functionName, "ShamrockInitialize");
    if (status) return;
    error = ShamrockGetNumberDevices(&numDevices);
    status = checkError(error, functionName, "ShamrockGetNumberDevices");
    if (status) return;
    if (numDevices < 1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  No Shamrock spectrographs found, numDevices=%d\n",
            driverName, functionName, numDevices);
    }
    return;
}

asynStatus shamrock::updateInitialPVs()
{
    static const char *functionName = "updateInitialPVs";
    asynStatus status;
    int error;
    char serial[64];
    float minWavelength, maxWavelength;
    int numGratings;
    int i;
    int numFlipperStatus;
    int present;

    // get device serial number
    ShamrockGetSerialNumber(shamrockId_, serial);
    setStringParam(SRSerialNumber_, serial);

    // Determine which slits are present
    for (i=SHAMROCK_SLIT_INDEX_MIN; i<=SHAMROCK_SLIT_INDEX_MAX; i++) {
        error = ShamrockAutoSlitIsPresent(shamrockId_, i, &present);
        status = checkError(error, functionName, "ShamrockAutoSlitIsPresent");
        slitIsPresent_[i-1] = (present == 1);
        setIntegerParam(i, SRSlitExists_, slitIsPresent_[i-1]);
    }

    // Determine how many gratings are present
    error = ShamrockGetNumberGratings(shamrockId_, &numGratings);
    status = checkError(error, functionName, "ShamrockGetNumberGratings");
    setIntegerParam(SRNumGratings_, numGratings);

    // Get wavelength range of each grating
    for (i=SHAMROCK_GRATINGMIN; i<=numGratings; i++) {
        setIntegerParam(i, SRGratingExists_, 1);
        error = ShamrockGetWavelengthLimits(shamrockId_, i, &minWavelength, &maxWavelength);
        status = checkError(error, functionName, "ShamrockGetWavelengthLimits");
        setDoubleParam(i, SRMinGratingWavelength_, minWavelength);
        setDoubleParam(i, SRMaxGratingWavelength_, maxWavelength);
    }
    for (i=numGratings+1; i<=MAX_GRATINGS; i++) {
        setIntegerParam(i, SRGratingExists_, 0);
    }

    // Determine which Flipper Mirrors exist
    for (i=SHAMROCK_FLIPPER_INDEX_MIN; i<=SHAMROCK_FLIPPER_INDEX_MAX; i++) {
        error = ShamrockFlipperMirrorIsPresent(shamrockId_, i, &numFlipperStatus);
        status = checkError(error, functionName, "ShamrockFlipperMirrorIsPresent");
        flipperMirrorIsPresent_[i-1] = (numFlipperStatus== 1); 
        setIntegerParam(i, SRFlipperMirrorExists_, flipperMirrorIsPresent_[i-1]);
    }

    // Get the wavelength limits for the detector ports
    for (i=SHAMROCK_PORTMIN; i<=SHAMROCK_PORTMAX; i++) {
        if (i == SHAMROCK_SIDE_PORT && !flipperMirrorIsPresent_[i]) continue;
        error = ShamrockGetCCDLimits(shamrockId_, i, &minWavelength, &maxWavelength);
        status = checkError(error, functionName, "ShamrockGetCCDLimits");
        setDoubleParam(i, SRMinCamWavelength_, minWavelength);
        setDoubleParam(i, SRMaxCamWavelength_, maxWavelength);
    }

    for (i=0; i<MAX_ADDR; i++) {
        callParamCallbacks(i);
    }
    status = getStatus();
    return status;
}

inline asynStatus shamrock::checkError(int status, const char *functionName, const char *shamrockFunction)
{
    if (status != SHAMROCK_SUCCESS) {
        ShamrockGetFunctionReturnDescription(status, lastError_, sizeof(lastError_));
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: ERROR calling %s Description=%s\n",
            driverName, functionName, shamrockFunction, lastError_);
        return asynError;
    }
    return asynSuccess;
}

asynStatus shamrock::getStatus()
{
    int error;
    asynStatus status;
    int grating;
    float wavelength;
    float width;
    int i;
    static const char *functionName = "getStatus";
    int port;

    //Get Flipper Status
    for (i=SHAMROCK_FLIPPER_INDEX_MIN; i<=SHAMROCK_FLIPPER_INDEX_MAX; i++) {
        if (flipperMirrorIsPresent_[i-1] == 0) continue;
        error = ShamrockGetFlipperMirror(shamrockId_, i, &port);
        status = checkError(error, functionName, "ShamrockGetFlipperMirror");
        if (status) return asynError;
        setIntegerParam(i, SRFlipperMirrorPort_, port);
    }

    error = ShamrockGetGrating(shamrockId_, &grating);
    status = checkError(error, functionName, "ShamrockGetGrating");
    if (status) return asynError;
    setIntegerParam(SRGrating_, grating);

    error = ShamrockGetWavelength(shamrockId_, &wavelength);
    status = checkError(error, functionName, "ShamrockGetWavelength");
    if (status) return asynError;
    setDoubleParam(SRWavelength_, wavelength);

    for (i=SHAMROCK_SLIT_INDEX_MIN; i<=SHAMROCK_SLIT_INDEX_MAX; i++) {
        setDoubleParam(i, SRSlitSize_, 0.);
        if (slitIsPresent_[i-1] == 0) continue;
        error = ShamrockGetAutoSlitWidth(shamrockId_, i, &width);
        status = checkError(error, functionName, "ShamrockGetAutoSlitWidth");
        if (status) return asynError;
        setDoubleParam(i, SRSlitSize_, width);
    }

    for (i=0; i<MAX_ADDR; i++) {
        callParamCallbacks(i);
    }
    return status;
}

/**
 * Sets calibration values for the shamrock based on the selected camera's pixel and sensor width.
 * \param[in] port specifies which output port the camera we are calibrating for is connected to.
 */
asynStatus shamrock::calibrate(int port)
{
    asynStatus status;
    int error;
    int camSensorWidth;
    double camPixelWidth;
    float pixelWidth;
    static const char *functionName = "calibrate";

    status = this->getIntegerParam(port, SRCamSensorWidth_, &camSensorWidth);
    if (status > asynSuccess) return status;
    status = this->getDoubleParam(port, SRCamPixelWidth_, &camPixelWidth);
    if (status > asynSuccess) return status;

    //Sets the number of pixels for calibration purposes
    error = ShamrockSetNumberPixels(shamrockId_, camSensorWidth);
    status = checkError(error, functionName, "ShamrockSetNumberPixels");

    //Set the pixel width in microns for calibration purposes.
    error = ShamrockSetPixelWidth(shamrockId_, camPixelWidth);
    status = checkError(error, functionName, "ShamrockSetPixelWidth");

    // Determine the number of pixels on the attached detector and the pixel size
    error = ShamrockGetNumberPixels(shamrockId_, &numPixels_);
    status = checkError(error, functionName, "ShamrockGetNumberPixels");
    error = ShamrockGetPixelWidth(shamrockId_, &pixelWidth);
    status = checkError(error, functionName, "ShamrockGetPixelWidth");
    calibration_ = (float *)calloc(numPixels_, sizeof(float));

    error = ShamrockGetCalibration(shamrockId_, calibration_, numPixels_);
    status = checkError(error, functionName, "ShamrockGetCalibration");
    if (status) return asynError;
    setDoubleParam(SRMinWavelength_, calibration_[0]);
    setDoubleParam(SRMaxWavelength_, calibration_[numPixels_-1]);

    doCallbacksFloat32Array(calibration_, numPixels_, SRCalibration_, 0);
    return status;
}

/** Sets an int32 parameter.
  * \param[in] pasynUser asynUser structure that contains the function code in pasynUser->reason. 
  * \param[in] value The value for this parameter 
  *
  * Takes action if the function code requires it.  ADAcquire, ADSizeX, and many other
  * function codes make calls to the Firewire library from this function. */
asynStatus shamrock::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    asynStatus status = asynSuccess;
    int error;
    int function = pasynUser->reason;
    int addr;
    int port;
    static const char *functionName = "writeInt32";

    pasynManager->getAddr(pasynUser, &addr);
    if (addr < 0) addr=0;

    /* Set the value in the parameter library.  This may change later but that's OK */
    status = setIntegerParam(addr, function, value);

    if (function == SRGrating_) {
        error = ShamrockSetGrating(shamrockId_, value);
        status = checkError(error, functionName, "ShamrockSetGrating");
        status = this->getIntegerParam(SHAMROCK_OUTPUT_FLIPPER, SRFlipperMirrorPort_, &port);
        status = this->calibrate(port);
    }
    else if (function == SRFlipperMirrorPort_) {
        if (flipperMirrorIsPresent_[addr-1]) {
            error = ShamrockSetFlipperMirror(shamrockId_, addr, value);
            status = checkError(error, functionName, "ShamrockSetFlipperMirror");
        }
        if (addr == SHAMROCK_OUTPUT_FLIPPER){
            this->calibrate(value);
        }
    }
    else if (function == SRCamSensorWidth_) {
        status = this->getIntegerParam(SHAMROCK_OUTPUT_FLIPPER, SRFlipperMirrorPort_, &port);
        if (port==addr) {
            this->calibrate(port);
        }
    }

    getStatus();

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
        "%s::%s function=%d, value=%d, status=%d\n",
        driverName, functionName, function, value, status);

    callParamCallbacks(addr);
    return status;
}

/** Sets an float64 parameter.
  * \param[in] pasynUser asynUser structure that contains the function code in pasynUser->reason. 
  * \param[in] value The value for this parameter 
  *
  * Takes action if the function code requires it. */
asynStatus shamrock::writeFloat64( asynUser *pasynUser, epicsFloat64 value)
{
    asynStatus status = asynSuccess;
    int error;
    int function = pasynUser->reason;
    int addr;
    int port;
    static const char *functionName = "writeFloat64";
    
    pasynManager->getAddr(pasynUser, &addr);
    if (addr < 0) addr=0;

    /* Set the value in the parameter library.  This may change later but that's OK */
    status = setDoubleParam(addr, function, value);

    if (function == SRWavelength_) {
        error = ShamrockSetWavelength(shamrockId_, (float) value);
        status = checkError(error, functionName, "ShamrockSetWavelength");
        status = this->getIntegerParam(SHAMROCK_OUTPUT_FLIPPER, SRFlipperMirrorPort_, &port);
        status = this->calibrate(port);
    } 
    else if (function == SRSlitSize_) {
        if (slitIsPresent_[addr-1]) {
          error = ShamrockSetAutoSlitWidth(shamrockId_, addr, (float) value);
          status = checkError(error, functionName, "ShamrockSetSlit");
        }
    }
    else if (function == SRCamPixelWidth_) {
        status = this->getIntegerParam(SHAMROCK_OUTPUT_FLIPPER, SRFlipperMirrorPort_, &port);
        if (port==addr) {
            status = this->calibrate(port);
        }
    }

    getStatus();

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s::%s function=%d, value=%f, status=%d\n",
        driverName, functionName, function, value, status);
    callParamCallbacks(addr);
    return status;
}

/** Reads float32 array.
  * \param[in] pasynUser asynUser structure that contains the function code in pasynUser->reason. 
  * \param[in] pValue The array
  * \param[in] nElements The size of the array
  * \param[out] nIn The number of elements copied to the array
  * Takes action if the function code requires it. */
asynStatus shamrock::readFloat32Array(asynUser *pasynUser, epicsFloat32 *pValue, size_t nElements, size_t *nIn)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    //static const char *functionName = "readFloat32Array";
    
    if (function == SRCalibration_) {
        *nIn = numPixels_;
        if (nElements < *nIn) *nIn = nElements;
        memcpy(pValue, calibration_, *nIn * sizeof(float));
    } 
    return status;
}

/** Print out a report; calls asynPortDriver::report to get base class report as well.
  * \param[in] fp File pointer to write output to
  * \param[in] details Level of detail desired.
 */
void shamrock::report(FILE *fp, int details)
{
    //static const char *functionName = "report";
    
    asynPortDriver::report(fp, details);
    return;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"shamrockId", iocshArgInt};
static const iocshArg configArg2 = {"iniPath", iocshArgStringPath};
static const iocshArg configArg3 = {"priority", iocshArgInt};
static const iocshArg configArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4};
static const iocshFuncDef configShamrock = {"shamrockConfig", 5, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    shamrockConfig(args[0].sval, args[1].ival, args[2].sval, 
                    args[3].ival, args[4].ival);
}


static void shamrockRegister(void)
{
    iocshRegister(&configShamrock, configCallFunc);
}

extern "C" {
epicsExportRegistrar(shamrockRegister);
}

