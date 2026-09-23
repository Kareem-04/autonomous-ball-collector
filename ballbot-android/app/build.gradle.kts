plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.kareem.ballbot"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.kareem.ballbot"
        minSdk = 28
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        // Honor 400 is arm64-v8a. Limiting ABI filters strips the x86/x86_64
        // TFLite native libs from the APK — reduces app size by ~30MB and
        // speeds up install/launch.
        ndk {
            abiFilters += listOf("arm64-v8a")
        }
    }

    buildTypes {
        debug {
            // Keep debug symbols for native crash reports
        }
        release {
            isMinifyEnabled = true                         // enables R8 (was false — turn it on for release)
            isShrinkResources = true
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        viewBinding = true
    }

    packaging {
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
        }
        // Prevent duplicate TFLite native libs from conflicting
        jniLibs {
            pickFirsts += listOf("**/libtensorflowlite_jni.so", "**/libtensorflowlite_gpu_jni.so")
        }
    }
}

dependencies {
    val cameraxVersion = "1.4.2"

    implementation("androidx.core:core-ktx:1.15.0")
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("com.google.android.material:material:1.12.0")
    implementation("androidx.constraintlayout:constraintlayout:2.2.1")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.7")
    implementation("androidx.activity:activity-ktx:1.10.1")

    implementation("androidx.camera:camera-core:$cameraxVersion")
    implementation("androidx.camera:camera-camera2:$cameraxVersion")
    implementation("androidx.camera:camera-lifecycle:$cameraxVersion")
    implementation("androidx.camera:camera-view:$cameraxVersion")

    // TFLite — tensorflow-lite includes NNAPI delegate (NnApiDelegate) built-in.
    // tensorflow-lite-gpu adds the GPU (OpenGL ES compute) delegate.
    // No extra NNAPI dependency needed.
//    implementation("org.tensorflow:tensorflow-lite:2.16.1")
//    implementation("org.tensorflow:tensorflow-lite-gpu:2.16.1")
//    implementation("org.tensorflow:tensorflow-lite-support:0.4.4")


    implementation("com.google.ai.edge.litert:litert:1.0.1")
    implementation("com.google.ai.edge.litert:litert-gpu:1.0.1")
    implementation("com.google.ai.edge.litert:litert-support:1.0.1")

    implementation("com.github.mik3y:usb-serial-for-android:3.9.0")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.9.0")

    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.2.1")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.6.1")
}