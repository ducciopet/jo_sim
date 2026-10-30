auto_scale_lr = dict(base_batch_size=16, enable=False)
backend_args = None
class_names = [
    'Car',
    'Pedestrian',
    'Cyclist',
]
data_prefix = dict(img='', pts='', sweeps='')
data_root = 'data/CODA/'
dataset_type = 'CodaDataset'
db_sampler = dict(
    classes=[
        'Car',
        'Pedestrian',
        'Cyclist',
    ],
    data_root='data/CODA/',
    info_path='data/CODA/coda_dbinfos_train.pkl',
    points_loader=dict(
        coord_type='LIDAR',
        load_dim=4,
        type='LoadPointsFromFile',
        use_dim=[
            0,
            1,
            2,
            3,
        ]),
    prepare=dict(
        filter_by_difficulty=[
            -1,
        ],
        filter_by_min_points=dict(Car=5, Cyclist=5, Pedestrian=5)),
    rate=1.0,
    sample_groups=dict(Car=3, Cyclist=4, Pedestrian=2),
    type='UnifiedDataBaseSampler')
default_hooks = dict(
    checkpoint=dict(interval=-1, type='CheckpointHook'),
    logger=dict(interval=50, type='LoggerHook'),
    param_scheduler=dict(type='ParamSchedulerHook'),
    sampler_seed=dict(type='DistSamplerSeedHook'),
    timer=dict(type='IterTimerHook'),
    visualization=dict(type='Det3DVisualizationHook'))
default_scope = 'mmdet3d'
env_cfg = dict(
    cudnn_benchmark=False,
    dist_cfg=dict(backend='nccl'),
    mp_cfg=dict(mp_start_method='fork', opencv_num_threads=0))
input_modality = dict(use_camera=True, use_lidar=True)
launcher = 'pytorch'
load_from = None
log_level = 'INFO'
log_processor = dict(by_epoch=True, type='LogProcessor', window_size=50)
lr = 2.5e-05
metainfo = dict(classes=[
    'Car',
    'Pedestrian',
    'Cyclist',
])
model = dict(
    data_preprocessor=dict(
        type='Det3DDataPreprocessor',
        voxel=True,
        voxel_layer=dict(
            max_num_points=10,
            max_voxels=(
                90000,
                120000,
            ),
            point_cloud_range=[
                -21.0,
                -21.0,
                -2.0,
                21.0,
                21.0,
                6.0,
            ],
            voxel_size=[
                0.075,
                0.075,
                0.2,
            ])),
    pts_backbone=dict(
        conv_cfg=dict(bias=False, type='Conv2d'),
        in_channels=256,
        layer_nums=[
            5,
            5,
        ],
        layer_strides=[
            1,
            2,
        ],
        norm_cfg=dict(eps=0.001, momentum=0.01, type='BN'),
        out_channels=[
            128,
            256,
        ],
        type='SECOND'),
    pts_bbox_head=dict(
        bbox_coder=dict(
            code_size=7,
            max_num=500,
            out_size_factor=8,
            pc_range=[
                -21.0,
                -21.0,
            ],
            post_center_range=[
                -30.0,
                -30.0,
                -10.0,
                30.0,
                30.0,
                10.0,
            ],
            score_threshold=0.1,
            type='CenterPointBBoxCoder',
            voxel_size=[
                0.075,
                0.075,
            ]),
        common_heads=dict(
            dim=(
                3,
                2,
            ), height=(
                1,
                2,
            ), reg=(
                2,
                2,
            ), rot=(
                2,
                2,
            )),
        in_channels=512,
        loss_bbox=dict(
            loss_weight=0.25, reduction='mean', type='mmdet.L1Loss'),
        loss_cls=dict(reduction='mean', type='mmdet.GaussianFocalLoss'),
        norm_bbox=True,
        separate_head=dict(
            final_kernel=3, init_bias=-2.19, type='SeparateHead'),
        share_conv_channel=64,
        tasks=[
            dict(class_names=[
                'Car',
            ], num_class=1),
            dict(class_names=[
                'Pedestrian',
            ], num_class=1),
            dict(class_names=[
                'Cyclist',
            ], num_class=1),
        ],
        type='CenterHead_coda'),
    pts_middle_encoder=dict(
        block_type='basicblock',
        encoder_channels=(
            (
                16,
                16,
                32,
            ),
            (
                32,
                32,
                64,
            ),
            (
                64,
                64,
                128,
            ),
            (
                128,
                128,
            ),
        ),
        encoder_paddings=(
            (
                0,
                0,
                1,
            ),
            (
                0,
                0,
                1,
            ),
            (
                0,
                0,
                [
                    0,
                    1,
                    1,
                ],
            ),
            (
                0,
                0,
            ),
        ),
        in_channels=4,
        order=(
            'conv',
            'norm',
            'act',
        ),
        output_channels=128,
        sparse_shape=[
            41,
            560,
            560,
        ],
        type='SparseEncoder'),
    pts_neck=dict(
        in_channels=[
            128,
            256,
        ],
        norm_cfg=dict(eps=0.001, momentum=0.01, type='BN'),
        out_channels=[
            256,
            256,
        ],
        type='SECONDFPN',
        upsample_cfg=dict(bias=False, type='deconv'),
        upsample_strides=[
            1,
            2,
        ],
        use_conv_for_no_stride=True),
    pts_voxel_encoder=dict(num_features=4, type='HardSimpleVFE'),
    test_cfg=dict(
        pts=dict(
            max_per_img=500,
            max_pool_nms=False,
            min_radius=[
                4,
                12,
                10,
                1,
                0.85,
                0.175,
            ],
            nms_thr=0.2,
            nms_type='rotate',
            out_size_factor=8,
            pc_range=[
                -21.0,
                -21.0,
            ],
            post_center_limit_range=[
                -30.0,
                -30.0,
                -10.0,
                30.0,
                30.0,
                10.0,
            ],
            post_max_size=83,
            pre_max_size=1000,
            score_threshold=0.1,
            voxel_size=[
                0.075,
                0.075,
            ])),
    train_cfg=dict(
        pts=dict(
            code_weights=[
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
            ],
            dense_reg=1,
            gaussian_overlap=0.1,
            grid_size=[
                560,
                560,
                40,
            ],
            max_objs=500,
            min_radius=2,
            out_size_factor=8,
            point_cloud_range=[
                -21.0,
                -21.0,
                -2.0,
                21.0,
                21.0,
                6.0,
            ],
            voxel_size=[
                0.075,
                0.075,
                0.2,
            ])),
    type='CenterPoint')
optim_wrapper = dict(
    clip_grad=dict(max_norm=35, norm_type=2),
    optimizer=dict(lr=2.5e-05, type='AdamW', weight_decay=0.01),
    type='OptimWrapper')
param_scheduler = [
    dict(
        T_max=4,
        begin=0,
        by_epoch=True,
        convert_to_iter_based=True,
        end=4,
        eta_min=0.00015000000000000001,
        type='CosineAnnealingLR'),
    dict(
        T_max=16,
        begin=4,
        by_epoch=True,
        convert_to_iter_based=True,
        end=20,
        eta_min=2.5e-09,
        type='CosineAnnealingLR'),
    dict(
        T_max=4,
        begin=0,
        by_epoch=True,
        convert_to_iter_based=True,
        end=4,
        eta_min=0.8947368421052632,
        type='CosineAnnealingMomentum'),
    dict(
        T_max=16,
        begin=4,
        by_epoch=True,
        convert_to_iter_based=True,
        end=20,
        eta_min=1,
        type='CosineAnnealingMomentum'),
]
point_cloud_range = [
    -21.0,
    -21.0,
    -2.0,
    21.0,
    21.0,
    6.0,
]
resume = False
test_cfg = dict()
test_dataloader = dict(
    batch_size=1,
    dataset=dict(
        ann_file='coda_infos_val.pkl',
        backend_args=None,
        box_type_3d='LiDAR',
        data_prefix=dict(img='', pts='', sweeps=''),
        data_root='data/CODA/',
        metainfo=dict(classes=[
            'Car',
            'Pedestrian',
            'Cyclist',
        ]),
        modality=dict(use_camera=True, use_lidar=True),
        pipeline=[
            dict(
                backend_args=None,
                coord_type='LIDAR',
                load_dim=4,
                type='LoadPointsFromFile',
                use_dim=4),
            dict(
                flip=False,
                img_scale=(
                    1333,
                    800,
                ),
                pts_scale_ratio=1,
                transforms=[
                    dict(
                        rot_range=[
                            0,
                            0,
                        ],
                        scale_ratio_range=[
                            1.0,
                            1.0,
                        ],
                        translation_std=[
                            0,
                            0,
                            0,
                        ],
                        type='GlobalRotScaleTrans'),
                    dict(type='RandomFlip3D'),
                    dict(
                        point_cloud_range=[
                            -21.0,
                            -21.0,
                            -2.0,
                            21.0,
                            21.0,
                            6.0,
                        ],
                        type='PointsRangeFilter'),
                ],
                type='MultiScaleFlipAug3D'),
            dict(keys=[
                'points',
            ], type='Pack3DDetInputs'),
        ],
        test_mode=True,
        type='CodaDataset'),
    drop_last=False,
    num_workers=4,
    persistent_workers=True,
    sampler=dict(shuffle=False, type='DefaultSampler'))
test_evaluator = dict(
    ann_file='data/CODA/coda_infos_val.pkl',
    backend_args=None,
    metric='bbox',
    type='CodaMetric')
test_pipeline = [
    dict(
        backend_args=None,
        coord_type='LIDAR',
        load_dim=4,
        type='LoadPointsFromFile',
        use_dim=4),
    dict(
        flip=False,
        img_scale=(
            1333,
            800,
        ),
        pts_scale_ratio=1,
        transforms=[
            dict(
                rot_range=[
                    0,
                    0,
                ],
                scale_ratio_range=[
                    1.0,
                    1.0,
                ],
                translation_std=[
                    0,
                    0,
                    0,
                ],
                type='GlobalRotScaleTrans'),
            dict(type='RandomFlip3D'),
            dict(
                point_cloud_range=[
                    -21.0,
                    -21.0,
                    -2.0,
                    21.0,
                    21.0,
                    6.0,
                ],
                type='PointsRangeFilter'),
        ],
        type='MultiScaleFlipAug3D'),
    dict(keys=[
        'points',
    ], type='Pack3DDetInputs'),
]
train_cfg = dict(by_epoch=True, max_epochs=20, val_interval=1)
train_dataloader = dict(
    batch_size=4,
    dataset=dict(
        dataset=dict(
            ann_file='coda_infos_train.pkl',
            box_type_3d='LiDAR',
            data_prefix=dict(img='', pts='', sweeps=''),
            data_root='data/CODA/',
            metainfo=dict(classes=[
                'Car',
                'Pedestrian',
                'Cyclist',
            ]),
            modality=dict(use_camera=True, use_lidar=True),
            pipeline=[
                dict(
                    backend_args=None,
                    coord_type='LIDAR',
                    load_dim=4,
                    type='LoadPointsFromFile',
                    use_dim=4),
                dict(
                    type='LoadAnnotations3D',
                    with_bbox_3d=True,
                    with_label_3d=True),
                dict(
                    rot_range=[
                        -0.3925,
                        0.3925,
                    ],
                    scale_ratio_range=[
                        0.95,
                        1.05,
                    ],
                    translation_std=[
                        0,
                        0,
                        0,
                    ],
                    type='GlobalRotScaleTrans'),
                dict(
                    flip_ratio_bev_horizontal=0.5,
                    flip_ratio_bev_vertical=0.5,
                    sync_2d=False,
                    type='RandomFlip3D'),
                dict(
                    point_cloud_range=[
                        -21.0,
                        -21.0,
                        -2.0,
                        21.0,
                        21.0,
                        6.0,
                    ],
                    type='PointsRangeFilter'),
                dict(
                    point_cloud_range=[
                        -21.0,
                        -21.0,
                        -2.0,
                        21.0,
                        21.0,
                        6.0,
                    ],
                    type='ObjectRangeFilter'),
                dict(
                    classes=[
                        'Car',
                        'Pedestrian',
                        'Cyclist',
                    ],
                    type='ObjectNameFilter'),
                dict(type='PointShuffle'),
                dict(
                    keys=[
                        'points',
                        'gt_bboxes_3d',
                        'gt_labels_3d',
                    ],
                    type='Pack3DDetInputs'),
            ],
            test_mode=False,
            type='CodaDataset',
            use_valid_flag=True),
        type='CBGSDataset'),
    num_workers=4,
    persistent_workers=True,
    sampler=dict(shuffle=True, type='DefaultSampler'))
train_pipeline = [
    dict(
        backend_args=None,
        coord_type='LIDAR',
        load_dim=4,
        type='LoadPointsFromFile',
        use_dim=4),
    dict(type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True),
    dict(
        rot_range=[
            -0.3925,
            0.3925,
        ],
        scale_ratio_range=[
            0.95,
            1.05,
        ],
        translation_std=[
            0,
            0,
            0,
        ],
        type='GlobalRotScaleTrans'),
    dict(
        flip_ratio_bev_horizontal=0.5,
        flip_ratio_bev_vertical=0.5,
        sync_2d=False,
        type='RandomFlip3D'),
    dict(
        point_cloud_range=[
            -21.0,
            -21.0,
            -2.0,
            21.0,
            21.0,
            6.0,
        ],
        type='PointsRangeFilter'),
    dict(
        point_cloud_range=[
            -21.0,
            -21.0,
            -2.0,
            21.0,
            21.0,
            6.0,
        ],
        type='ObjectRangeFilter'),
    dict(classes=[
        'Car',
        'Pedestrian',
        'Cyclist',
    ], type='ObjectNameFilter'),
    dict(type='PointShuffle'),
    dict(
        keys=[
            'points',
            'gt_bboxes_3d',
            'gt_labels_3d',
        ],
        type='Pack3DDetInputs'),
]
val_cfg = dict()
val_dataloader = dict(
    batch_size=4,
    dataset=dict(
        ann_file='coda_infos_val.pkl',
        backend_args=None,
        box_type_3d='LiDAR',
        data_prefix=dict(img='', pts='', sweeps=''),
        data_root='data/CODA/',
        metainfo=dict(classes=[
            'Car',
            'Pedestrian',
            'Cyclist',
        ]),
        modality=dict(use_camera=True, use_lidar=True),
        pipeline=[
            dict(
                backend_args=None,
                coord_type='LIDAR',
                load_dim=4,
                type='LoadPointsFromFile',
                use_dim=4),
            dict(
                flip=False,
                img_scale=(
                    1333,
                    800,
                ),
                pts_scale_ratio=1,
                transforms=[
                    dict(
                        rot_range=[
                            0,
                            0,
                        ],
                        scale_ratio_range=[
                            1.0,
                            1.0,
                        ],
                        translation_std=[
                            0,
                            0,
                            0,
                        ],
                        type='GlobalRotScaleTrans'),
                    dict(type='RandomFlip3D'),
                    dict(
                        point_cloud_range=[
                            -21.0,
                            -21.0,
                            -2.0,
                            21.0,
                            21.0,
                            6.0,
                        ],
                        type='PointsRangeFilter'),
                ],
                type='MultiScaleFlipAug3D'),
            dict(keys=[
                'points',
            ], type='Pack3DDetInputs'),
        ],
        test_mode=True,
        type='CodaDataset'),
    drop_last=False,
    num_workers=4,
    persistent_workers=True,
    sampler=dict(shuffle=False, type='DefaultSampler'))
val_evaluator = dict(
    ann_file='data/CODA/coda_infos_val.pkl',
    backend_args=None,
    metric='bbox',
    type='CodaMetric')
vis_backends = [
    dict(type='LocalVisBackend'),
]
visualizer = dict(
    name='visualizer',
    type='Det3DLocalVisualizer',
    vis_backends=[
        dict(type='LocalVisBackend'),
    ])
voxel_size = [
    0.075,
    0.075,
    0.2,
]
work_dir = './work_dirs/centerpoint_coda'
