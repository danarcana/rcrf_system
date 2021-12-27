// This is an autogenerated config file, any changes to this file will be overwritten

#ifndef PSA_CRYPTO_CONFIG_AUTOGEN_H
#define PSA_CRYPTO_CONFIG_AUTOGEN_H

#define PSA_WANT_KEY_TYPE_AES
#define PSA_WANT_ALG_CCM
#define PSA_WANT_ALG_ECB_NO_PADDING
#define MBEDTLS_PSA_BUILTIN_ALG_ECB_NO_PADDING 1
#define PSA_WANT_ALG_SHA_224
#define PSA_WANT_ALG_SHA_256
#define MBEDTLS_PSA_ACCEL_ALG_SHA_1
#define MBEDTLS_PSA_ACCEL_ALG_SHA_224
#define MBEDTLS_PSA_ACCEL_ALG_SHA_256
#define MBEDTLS_PSA_ACCEL_KEY_TYPE_AES
#define MBEDTLS_PSA_ACCEL_ALG_ECB_NO_PADDING
#define MBEDTLS_PSA_ACCEL_ALG_CBC_NO_PADDING
#define MBEDTLS_PSA_ACCEL_ALG_CBC_PKCS7
#define MBEDTLS_PSA_ACCEL_ALG_CTR
#define MBEDTLS_PSA_ACCEL_ALG_CFB
#define MBEDTLS_PSA_ACCEL_ALG_OFB
#define MBEDTLS_PSA_ACCEL_ALG_GCM
#define MBEDTLS_PSA_ACCEL_ALG_CCM
#define MBEDTLS_PSA_ACCEL_ALG_CMAC

#define MBEDTLS_PSA_KEY_SLOT_COUNT (1 + SL_PSA_KEY_USER_SLOT_COUNT)
#define SL_PSA_ITS_MAX_FILES ( + SL_PSA_ITS_USER_MAX_FILES)

#endif
